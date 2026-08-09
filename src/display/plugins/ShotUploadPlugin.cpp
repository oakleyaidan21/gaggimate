#include "ShotUploadPlugin.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <WiFi.h>
#include <display/core/Controller.h>
#include <display/core/Event.h>
#include <display/models/shot_log_format.h>
#include <time.h>

namespace {
constexpr size_t UPLOAD_QUEUE_LEN = 8;
constexpr unsigned long WAKE_CACHE_MS = 10UL * 60UL * 1000UL;
constexpr uint32_t HTTP_TIMEOUT_MS = 90000;
constexpr int MAX_ATTEMPTS = 3;
constexpr uint32_t MAX_TIMESERIES_SAMPLES = 120;
} // namespace

ShotUploadPlugin ShotUpload;

void ShotUploadPlugin::setup(Controller *c, PluginManager *pluginManager) {
    controller = c;
    uploadQueue = xQueueCreate(UPLOAD_QUEUE_LEN, sizeof(char *));
    xTaskCreatePinnedToCore(uploadTask, "ShotUpload", configMINIMAL_STACK_SIZE * 8, this, 1, nullptr, 0);

    pluginManager->on("controller:wifi:connect", [this](Event const &) {
        if (controller->getSettings().isShotUploadActive()) {
            enqueueWake();
        }
    });

    pluginManager->on("shot:completed", [this](Event const &event) {
        String id = event.getString("id");
        if (id.length() > 0) {
            pendingShotId = id;
            ESP_LOGI("ShotUpload", "Shot %s ready for manual upload", id.c_str());
        }
    });
}

void ShotUploadPlugin::uploadPending() {
    if (!controller->getSettings().isShotUploadActive() || pendingShotId.isEmpty()) {
        return;
    }
    enqueueUpload(pendingShotId);
}

void ShotUploadPlugin::enqueueUpload(const String &shotId) {
    if (uploadQueue == nullptr || shotId.isEmpty()) {
        return;
    }
    char *copy = strdup(shotId.c_str());
    if (copy == nullptr) {
        ESP_LOGE("ShotUpload", "Out of memory queuing shot %s", shotId.c_str());
        return;
    }
    if (xQueueSend(uploadQueue, &copy, 0) != pdTRUE) {
        free(copy);
        ESP_LOGW("ShotUpload", "Upload queue full; dropping shot %s", shotId.c_str());
    }
}

void ShotUploadPlugin::enqueueWake() {
    if (uploadQueue == nullptr) {
        return;
    }
    // Empty string is the wake-only sentinel (real shot ids are never empty).
    char *copy = strdup("");
    if (copy == nullptr) {
        ESP_LOGE("ShotUpload", "Out of memory queuing wake");
        return;
    }
    if (xQueueSend(uploadQueue, &copy, 0) != pdTRUE) {
        free(copy);
        ESP_LOGW("ShotUpload", "Upload queue full; dropping wake");
    }
}

String ShotUploadPlugin::wakeUrlFromUploadUrl(const String &uploadUrl) const {
    int schemeEnd = uploadUrl.indexOf("://");
    if (schemeEnd < 0) {
        return "";
    }
    int pathStart = uploadUrl.indexOf('/', schemeEnd + 3);
    if (pathStart < 0) {
        return uploadUrl + "/up";
    }
    return uploadUrl.substring(0, pathStart) + "/up";
}

void ShotUploadPlugin::wakeServer() {
    Settings &settings = controller->getSettings();
    String wakeUrl = wakeUrlFromUploadUrl(settings.getShotUploadUrl());
    if (wakeUrl.isEmpty() || WiFi.status() != WL_CONNECTED) {
        return;
    }

    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    if (!http.begin(wakeUrl)) {
        ESP_LOGW("ShotUpload", "Wake begin failed for %s", wakeUrl.c_str());
        return;
    }
    int code = http.GET();
    http.end();
    if (code >= 200 && code < 300) {
        lastWakeMs = millis();
        ESP_LOGI("ShotUpload", "Wake OK (%d) %s", code, wakeUrl.c_str());
    } else {
        ESP_LOGW("ShotUpload", "Wake failed (%d) %s", code, wakeUrl.c_str());
    }
}

String ShotUploadPlugin::buildPayload(const String &shotId) {
    fs::FS *fs = controller->isSDCard() ? static_cast<fs::FS *>(&SD_MMC) : static_cast<fs::FS *>(&LittleFS);
    String slogPath = "/h/" + shotId + ".slog";
    if (!fs->exists(slogPath)) {
        ESP_LOGE("ShotUpload", "Missing slog for %s", shotId.c_str());
        return "";
    }

    File file = fs->open(slogPath, "r");
    if (!file) {
        ESP_LOGE("ShotUpload", "Could not open slog for %s", shotId.c_str());
        return "";
    }

    ShotLogHeader header{};
    size_t nread = file.read(reinterpret_cast<uint8_t *>(&header), sizeof(header));
    if (nread != sizeof(header) || header.magic != SHOT_LOG_MAGIC) {
        file.close();
        ESP_LOGE("ShotUpload", "Bad slog header for %s", shotId.c_str());
        return "";
    }

    // Resolve yield: BT scale header → notes doseOut → last sample BT/estimated weight.
    // header.finalWeight is only written from Bluetooth scale weight, so no-scale
    // shots need the sample/notes fallbacks.
    float yieldG = header.finalWeight > 0 ? static_cast<float>(header.finalWeight) / 10.0f : 0.0f;

    JsonDocument notes;
    String notesPath = "/h/" + shotId + ".json";
    if (fs->exists(notesPath)) {
        File nf = fs->open(notesPath, "r");
        if (nf) {
            deserializeJson(notes, nf);
            nf.close();
        }
    }
    if (yieldG <= 0.0f && !notes["doseOut"].isNull()) {
        if (notes["doseOut"].is<const char *>() || notes["doseOut"].is<String>()) {
            yieldG = notes["doseOut"].as<String>().toFloat();
        } else {
            yieldG = notes["doseOut"].as<float>();
        }
    }

    const size_t sampleSize = header.reserved0 > 0 ? header.reserved0 : SHOT_LOG_SAMPLE_SIZE;
    if (yieldG <= 0.0f && header.sampleCount > 0 && sampleSize == sizeof(ShotLogSample)) {
        const size_t offset = static_cast<size_t>(header.headerSize) + (header.sampleCount - 1) * sampleSize;
        ShotLogSample sample{};
        if (file.seek(offset) && file.read(reinterpret_cast<uint8_t *>(&sample), sizeof(sample)) == sizeof(sample)) {
            if (sample.v > 0) {
                yieldG = static_cast<float>(sample.v) / 10.0f;
            } else if (sample.ev > 0) {
                yieldG = static_cast<float>(sample.ev) / 10.0f;
            }
        }
    }

    JsonDocument doc;
    JsonObject shot = doc["shot"].to<JsonObject>();
    shot["external_id"] = shotId;
    shot["source"] = "gaggimate";
    if (header.startEpoch > 0) {
        time_t epoch = static_cast<time_t>(header.startEpoch);
        struct tm timeinfo {};
        gmtime_r(&epoch, &timeinfo);
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        shot["started_at"] = buf;
    }
    shot["duration_ms"] = static_cast<int>(header.durationMs);
    shot["profile_name"] = String(header.profileName);
    if (yieldG > 0.0f) {
        shot["yield"] = yieldG;
    }
    if (!notes["doseIn"].isNull()) {
        if (notes["doseIn"].is<const char *>() || notes["doseIn"].is<String>()) {
            shot["dose"] = notes["doseIn"].as<String>().toFloat();
        } else {
            shot["dose"] = notes["doseIn"].as<float>();
        }
    }
    if (!notes["grindSetting"].isNull()) {
        shot["grind_setting"] = notes["grindSetting"].as<const char *>();
    }
    if (!notes["taste"].isNull()) {
        shot["taste"] = notes["taste"].as<const char *>();
    }
    if (!notes["notes"].isNull()) {
        shot["notes"] = notes["notes"].as<const char *>();
    }
    if (!notes["rating"].isNull()) {
        shot["rating"] = notes["rating"].as<int>();
    }

    if (header.phaseTransitionCount > 0) {
        JsonArray phases = shot["phases"].to<JsonArray>();
        const uint8_t count = header.phaseTransitionCount > 12 ? 12 : header.phaseTransitionCount;
        for (uint8_t i = 0; i < count; ++i) {
            const PhaseTransition &tr = header.phaseTransitions[i];
            JsonObject phase = phases.add<JsonObject>();
            phase["sample_index"] = tr.sampleIndex;
            phase["phase_number"] = tr.phaseNumber;
            phase["name"] = String(tr.phaseName);
            phase["exit_reason"] = tr.transitionReason;
        }
    }

    if (header.sampleCount > 0 && sampleSize == sizeof(ShotLogSample)) {
        JsonObject timeseries = shot["timeseries"].to<JsonObject>();
        const uint16_t intervalMs = header.sampleInterval > 0 ? header.sampleInterval : SHOT_LOG_SAMPLE_INTERVAL_MS;
        timeseries["interval_ms"] = intervalMs;
        timeseries["final_exit_reason"] = header.finalExitReason;
        JsonArray samples = timeseries["samples"].to<JsonArray>();

        uint32_t step = 1;
        if (header.sampleCount > MAX_TIMESERIES_SAMPLES) {
            step = (header.sampleCount + MAX_TIMESERIES_SAMPLES - 1) / MAX_TIMESERIES_SAMPLES;
        }

        ShotLogSample sample{};
        for (uint32_t i = 0; i < header.sampleCount; i += step) {
            const size_t offset = static_cast<size_t>(header.headerSize) + i * sampleSize;
            if (!file.seek(offset) || file.read(reinterpret_cast<uint8_t *>(&sample), sizeof(sample)) != sizeof(sample)) {
                break;
            }
            JsonObject row = samples.add<JsonObject>();
            row["t"] = static_cast<int>(sample.t) * static_cast<int>(intervalMs);
            row["ct"] = static_cast<float>(sample.ct) / 10.0f;
            row["cp"] = static_cast<float>(sample.cp) / 10.0f;
            row["fl"] = static_cast<float>(sample.fl) / 100.0f;
            row["pf"] = static_cast<float>(sample.pf) / 100.0f;
            row["v"] = static_cast<float>(sample.v) / 10.0f;
            row["ev"] = static_cast<float>(sample.ev) / 10.0f;
        }
    }
    file.close();

    String out;
    serializeJson(doc, out);
    return out;
}

bool ShotUploadPlugin::postShot(const String &shotId) {
    Settings &settings = controller->getSettings();
    if (!settings.isShotUploadActive() || settings.getShotUploadUrl().isEmpty() || settings.getShotUploadToken().isEmpty()) {
        return false;
    }
    if (WiFi.status() != WL_CONNECTED) {
        ESP_LOGW("ShotUpload", "WiFi down; skip upload of %s", shotId.c_str());
        return false;
    }

    if (lastWakeMs == 0 || (millis() - lastWakeMs) > WAKE_CACHE_MS) {
        wakeServer();
    }

    String payload = buildPayload(shotId);
    if (payload.isEmpty()) {
        return false;
    }

    for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
        HTTPClient http;
        http.setTimeout(HTTP_TIMEOUT_MS);
        if (!http.begin(settings.getShotUploadUrl())) {
            ESP_LOGW("ShotUpload", "HTTP begin failed (attempt %d)", attempt);
            continue;
        }
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Authorization", "Bearer " + settings.getShotUploadToken());
        int code = http.POST(payload);
        http.end();
        if (code == 200 || code == 201) {
            ESP_LOGI("ShotUpload", "Uploaded shot %s (%d)", shotId.c_str(), code);
            lastWakeMs = millis();
            if (pendingShotId == shotId) {
                pendingShotId = "";
            }
            return true;
        }
        ESP_LOGW("ShotUpload", "Upload %s failed with %d (attempt %d)", shotId.c_str(), code, attempt);
        vTaskDelay(pdMS_TO_TICKS(500 * attempt));
    }
    return false;
}

void ShotUploadPlugin::uploadTask(void *arg) {
    auto *plugin = static_cast<ShotUploadPlugin *>(arg);
    while (true) {
        char *idPtr = nullptr;
        if (xQueueReceive(plugin->uploadQueue, &idPtr, portMAX_DELAY) == pdTRUE && idPtr != nullptr) {
            String id(idPtr);
            free(idPtr);
            if (id.isEmpty()) {
                plugin->wakeServer();
            } else {
                plugin->postShot(id);
            }
        }
    }
}
