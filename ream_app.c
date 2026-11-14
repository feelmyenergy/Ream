/*
 * REAM - RF Acoustic Eavesdropping Mapper
 * Flipper Zero SubGHz Application
 * 
 * PROPRIETARY - DO NOT DISTRIBUTE
 * Unauthorized copying or modification prohibited
 */

#include <furi.h>
#include <gui/gui.h>
#include <input/input.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <storage/storage.h>
#include <toolbox/stream.h>
#include <math.h>

#include "subghz/subghz_receiver.h"
#include "subghz/subghz_setting.h"

#define TAG "REAM"
#define SCAN_DURATION_MS 5000
#define MAX_SIGNALS 64
#define IQ_SAMPLE_RATE 250000
#define FFT_SIZE 256

typedef enum {
    ModUnknown,
    ModAM,
    ModFM,
    ModDigital,
    ModVideo,
} ModulationType;

typedef struct {
    uint32_t frequency;
    int8_t rssi;
    float entropy;
    ModulationType mod_type;
    uint32_t timestamp;
    float signal_variance;
    bool is_active; // Audio activity detected
} RFSignal;

typedef struct {
    RFSignal signals[MAX_SIGNALS];
    uint16_t signal_count;
    bool scanning;
    float min_rssi_threshold;
    uint32_t scan_start_time;
    FuriMutex* mutex;
} REAMState;

/* ============== SIGNAL PROCESSING ============== */

// Simplified entropy calculation (spectral analysis)
static float calculate_entropy(int16_t* iq_samples, size_t count) {
    if (count < 2) return 0.0f;
    
    float mean = 0;
    for (size_t i = 0; i < count; i++) {
        mean += abs(iq_samples[i]);
    }
    mean /= count;
    
    float variance = 0;
    for (size_t i = 0; i < count; i++) {
        float diff = abs(iq_samples[i]) - mean;
        variance += diff * diff;
    }
    variance /= count;
    
    // Normalized entropy (0-1 scale)
    // Higher entropy = more signal structure (likely modulated audio)
    return (variance / (mean * mean + 0.001f)) > 2.0f ? 1.0f : (variance / (mean * mean + 0.001f)) / 2.0f;
}

// Detect modulation type by signal characteristics
static ModulationType detect_modulation(int16_t* iq_samples, size_t count) {
    if (count < 16) return ModUnknown;
    
    // Analyze amplitude distribution
    float peak_amplitude = 0;
    float avg_amplitude = 0;
    
    for (size_t i = 0; i < count; i++) {
        float amp = fabsf((float)iq_samples[i]);
        if (amp > peak_amplitude) peak_amplitude = amp;
        avg_amplitude += amp;
    }
    avg_amplitude /= count;
    
    // Crest factor analysis
    float crest_factor = peak_amplitude / (avg_amplitude + 0.001f);
    
    // AM modulation has high crest factor
    if (crest_factor > 4.5f) {
        return ModAM;
    }
    
    // FM has more uniform envelope
    if (crest_factor > 2.0f && crest_factor <= 4.5f) {
        return ModFM;
    }
    
    // Digital has packet-like bursts
    if (crest_factor > 1.2f && crest_factor <= 2.0f) {
        return ModDigital;
    }
    
    // Video sync patterns (discrete timing)
    return ModVideo;
}

// Detect audio activity via modulation envelope variation
static bool detect_audio_activity(int16_t* iq_samples, size_t count, float baseline_variance) {
    if (count < 32) return false;
    
    float variance = 0;
    float mean = 0;
    
    for (size_t i = 0; i < count; i++) {
        mean += abs(iq_samples[i]);
    }
    mean /= count;
    
    for (size_t i = 0; i < count; i++) {
        float diff = abs(iq_samples[i]) - mean;
        variance += diff * diff;
    }
    variance /= count;
    
    // If variance spike > 40% above baseline, audio detected
    return variance > (baseline_variance * 1.4f);
}

/* ============== SCANNING ENGINE ============== */

static void ream_scan_callback(SubGhzReceiver* receiver, void* context) {
    REAMState* state = (REAMState*)context;
    if (!state || state->signal_count >= MAX_SIGNALS) return;
    
    furi_mutex_acquire(state->mutex, FuriWaitForever);
    
    // Get RSSI
    int8_t rssi = subghz_receiver_get_rssi(receiver);
    
    if (rssi > state->min_rssi_threshold) {
        uint32_t freq = subghz_receiver_get_frequency(receiver);
        
        // Check if frequency already logged
        bool found = false;
        for (uint16_t i = 0; i < state->signal_count; i++) {
            if (state->signals[i].frequency == freq) {
                if (rssi > state->signals[i].rssi) {
                    state->signals[i].rssi = rssi;
                }
                found = true;
                break;
            }
        }
        
        if (!found) {
            RFSignal new_signal = {
                .frequency = freq,
                .rssi = rssi,
                .entropy = 0.65f, // Placeholder
                .mod_type = ModDigital,
                .timestamp = furi_get_tick(),
                .signal_variance = 0.5f,
                .is_active = false,
            };
            state->signals[state->signal_count++] = new_signal;
        }
    }
    
    furi_mutex_release(state->mutex);
}

static void ream_start_scan(REAMState* state) {
    if (state->scanning) return;
    
    state->scanning = true;
    state->signal_count = 0;
    state->scan_start_time = furi_get_tick();
    state->min_rssi_threshold = -95; // Sensitive threshold
    
    FURI_LOG_I(TAG, "Scan started");
}

static void ream_stop_scan(REAMState* state) {
    state->scanning = false;
    FURI_LOG_I(TAG, "Scan stopped. Found %d signals", state->signal_count);
}

/* ============== DATA EXPORT ============== */

static bool ream_export_json(REAMState* state, const char* filepath) {
    if (!state || state->signal_count == 0) return false;
    
    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);
    
    if (!storage_file_open(file, filepath, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        storage_file_free(file);
        furi_record_close(RECORD_STORAGE);
        return false;
    }
    
    // Write JSON header
    storage_file_write_format(file, "{\n\"scan_metadata\": {\n");
    storage_file_write_format(file, "\"timestamp\": %lu,\n", state->scan_start_time);
    storage_file_write_format(file, "\"signal_count\": %d,\n", state->signal_count);
    storage_file_write_format(file, "\"min_rssi_threshold\": %.1f\n", state->min_rssi_threshold);
    storage_file_write_format(file, "},\n\"signals\": [\n");
    
    // Write each signal
    for (uint16_t i = 0; i < state->signal_count; i++) {
        RFSignal* sig = &state->signals[i];
        
        storage_file_write_format(file, "{\n");
        storage_file_write_format(file, "\"frequency_mhz\": %.3f,\n", sig->frequency / 1000000.0f);
        storage_file_write_format(file, "\"rssi_dbm\": %d,\n", sig->rssi);
        storage_file_write_format(file, "\"entropy\": %.4f,\n", sig->entropy);
        storage_file_write_format(file, "\"modulation\": \"%s\",\n",
            sig->mod_type == ModAM ? "AM" :
            sig->mod_type == ModFM ? "FM" :
            sig->mod_type == ModDigital ? "Digital" :
            sig->mod_type == ModVideo ? "Video" : "Unknown");
        storage_file_write_format(file, "\"audio_activity\": %s,\n", sig->is_active ? "true" : "false");
        storage_file_write_format(file, "\"signal_variance\": %.4f\n", sig->signal_variance);
        storage_file_write_format(file, "}%s\n", i < state->signal_count - 1 ? "," : "");
    }
    
    storage_file_write_format(file, "]\n}\n");
    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
    
    FURI_LOG_I(TAG, "Exported %d signals to %s", state->signal_count, filepath);
    return true;
}

/* ============== UI CALLBACKS ============== */

static void ream_input_callback(InputEvent* input_event, void* context) {
    FuriMessageQueue* queue = (FuriMessageQueue*)context;
    furi_message_queue_put(queue, input_event, FuriWaitForever);
}

static void ream_draw_callback(Canvas* canvas, void* context) {
    REAMState* state = (REAMState*)context;
    
    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 10, "REAM - RF Acoustic Mapper");
    
    canvas_set_font(canvas, FontSecondary);
    
    if (state->scanning) {
        canvas_draw_str(canvas, 2, 25, "Status: SCANNING");
        uint32_t elapsed = furi_get_tick() - state->scan_start_time;
        canvas_draw_str_aligned(canvas, 128, 25, AlignRight, AlignTop, 
            furi_string_printf(NULL, "%lds", elapsed / 1000));
    } else {
        canvas_draw_str(canvas, 2, 25, "Status: IDLE");
    }
    
    canvas_draw_str_aligned(canvas, 2, 40, AlignLeft, AlignTop, 
        furi_string_printf(NULL, "Signals: %d/%d", state->signal_count, MAX_SIGNALS));
    
    // Draw signal list
    uint8_t y_offset = 55;
    for (uint16_t i = 0; i < state->signal_count && i < 4; i++) {
        RFSignal* sig = &state->signals[i];
        canvas_draw_str_aligned(canvas, 2, y_offset, AlignLeft, AlignTop,
            furi_string_printf(NULL, "%3.1fMHz | %ddBm | %s%s",
                sig->frequency / 1000000.0f,
                sig->rssi,
                sig->mod_type == ModAM ? "AM" :
                sig->mod_type == ModFM ? "FM" :
                sig->mod_type == ModDigital ? "DIG" : "VID",
                sig->is_active ? "*" : ""));
        y_offset += 12;
    }
    
    // Footer
    canvas_draw_str(canvas, 2, 126, "[OK] Start  [Back] Menu");
}

/* ============== MAIN APP ============== */

int32_t ream_app(void* p) {
    (void)p;
    
    REAMState* state = malloc(sizeof(REAMState));
    state->signal_count = 0;
    state->scanning = false;
    state->min_rssi_threshold = -95;
    state->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    memset(state->signals, 0, sizeof(state->signals));
    
    // GUI setup
    Gui* gui = furi_record_open(RECORD_GUI);
    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, ream_draw_callback, state);
    
    FuriMessageQueue* input_queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    view_port_input_callback_set(view_port, ream_input_callback, input_queue);
    
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);
    
    // Main loop
    InputEvent input;
    bool running = true;
    
    while (running) {
        if (furi_message_queue_get(input_queue, &input, FuriWaitForever) == FuriStatusOk) {
            if (input.type == InputTypePress) {
                if (input.key == InputKeyOk) {
                    if (state->scanning) {
                        ream_stop_scan(state);
                    } else {
                        ream_start_scan(state);
                    }
                } else if (input.key == InputKeyBack) {
                    running = false;
                }
            }
        }
        
        view_port_update(view_port);
        furi_delay_ms(100);
    }
    
    // Cleanup
    ream_export_json(state, "/ext/ream_export.json");
    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_message_queue_free(input_queue);
    furi_mutex_free(state->mutex);
    free(state);
    furi_record_close(RECORD_GUI);
    
    return 0;
}
