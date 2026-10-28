// LVGL 8.4 HOME screen: average temperature / humidity (tap for each sensor)
// and the door sensor with open/close counts.
// Everything here runs in the LVGL task (ui_create inside the lock, the rest
// from lv_timer / widget event callbacks). Data comes only from the rs485_*
// and door_* snapshot functions, which copy under their own locks.
// The LVGL task has a 6 KB stack, so buffers are static and no float printf is used.

#include <Arduino.h>
#include <Preferences.h>
#include <lvgl.h>
#include "config.h"
#include "rs485_capture.h"
#include "door.h"
#include "buzzer.h"
#include "io_extension.h"
#include "ui.h"

// ---------- fonts: only those enabled in lv_conf.h (12, 14, 16, 26, 30) ----------
#if LV_FONT_MONTSERRAT_30
  #define FONT_XL (&lv_font_montserrat_30)
#elif LV_FONT_MONTSERRAT_26
  #define FONT_XL (&lv_font_montserrat_26)
#else
  #define FONT_XL LV_FONT_DEFAULT
#endif
#if LV_FONT_MONTSERRAT_26
  #define FONT_L (&lv_font_montserrat_26)
#elif LV_FONT_MONTSERRAT_16
  #define FONT_L (&lv_font_montserrat_16)
#else
  #define FONT_L LV_FONT_DEFAULT
#endif
#if LV_FONT_MONTSERRAT_16
  #define FONT_M (&lv_font_montserrat_16)
#else
  #define FONT_M LV_FONT_DEFAULT
#endif
#if LV_FONT_MONTSERRAT_14
  #define FONT_S (&lv_font_montserrat_14)
#else
  #define FONT_S LV_FONT_DEFAULT
#endif

#define COL_BG     lv_color_hex(0x0F1418)
#define COL_CARD   lv_color_hex(0x1B242B)
#define COL_PRESS  lv_color_hex(0x26323B)
#define COL_TEXT   lv_color_hex(0xE8EEF2)
#define COL_MUTED  lv_color_hex(0x7E8B94)
#define COL_OK     lv_color_hex(0x3DDC97)
#define COL_WARN   lv_color_hex(0xF5B841)
#define COL_IDLE   lv_color_hex(0x5A656D)
#define COL_TEMP   lv_color_hex(0xFF9F5A)
#define COL_HUM    lv_color_hex(0x4AB8FF)
#define COL_ALARM  lv_color_hex(0xFF4D4D)

#define BTN_H  64
#define DEG_C  "\xC2\xB0""C"

enum BusState { BS_GREEN, BS_AMBER, BS_GREY, BS_SCAN };

// ---------- widgets ----------
static lv_obj_t* s_bus_dot;
static lv_obj_t* s_bus_lbl;
static lv_obj_t* s_temp_val;
static lv_obj_t* s_temp_sub;
static lv_obj_t* s_hum_val;
static lv_obj_t* s_hum_sub;
static lv_obj_t* s_door_dot;
static lv_obj_t* s_door_state;
static lv_obj_t* s_door_since;
static lv_obj_t* s_door_opens;
static lv_obj_t* s_door_closes;
static lv_obj_t* s_silence_btn;      // only visible while the door-open alarm is buzzing
static lv_obj_t* s_detail;           // full-screen sensor list (on the top layer)
static lv_obj_t* s_detail_title;
static lv_obj_t* s_detail_table;
static bool      s_detail_temp;      // detail list shows temperature (true) or humidity (false)

static BusStats   s_st;
static DoorStatus s_door;
static char       s_txt[160];        // scratch text (LVGL task only)

// =====================================================================
//  helpers
// =====================================================================
static void fmt_duration(char* buf, size_t n, uint32_t ms) {
  const uint32_t s = ms / 1000;
  if (s < 60) snprintf(buf, n, "%lu s", (unsigned long)s);
  else if (s < 3600) snprintf(buf, n, "%lu min %lu s", (unsigned long)(s / 60), (unsigned long)(s % 60));
  else if (s < 86400) snprintf(buf, n, "%lu h %lu min", (unsigned long)(s / 3600), (unsigned long)(s % 3600 / 60));
  else snprintf(buf, n, "%lu d %lu h", (unsigned long)(s / 86400), (unsigned long)(s % 86400 / 3600));
}

// Fixed-point print without float printf (keeps the 6 KB LVGL stack safe).
static void fmt_fixed(char* buf, size_t n, float v, uint8_t decimals) {
  if (decimals > 6) decimals = 6;
  int64_t mul = 1;
  for (uint8_t i = 0; i < decimals; i++) mul *= 10;
  const bool neg = v < 0;
  const int64_t x = (int64_t)((neg ? -v : v) * (float)mul + 0.5f);
  if (decimals == 0) {
    snprintf(buf, n, "%s%lld", neg && x ? "-" : "", (long long)x);
  } else {
    char frac[8];
    int64_t f = x % mul;
    for (int i = decimals - 1; i >= 0; i--) { frac[i] = (char)('0' + f % 10); f /= 10; }
    frac[decimals] = 0;
    snprintf(buf, n, "%s%lld.%s", neg && x ? "-" : "", (long long)(x / mul), frac);
  }
}

static BusState bus_state(const BusStats& st, uint32_t now) {
  if (st.scanning) return BS_SCAN;
  if (st.last_valid_ms && now - st.last_valid_ms < STATUS_WINDOW_MS) return BS_GREEN;
  if (st.last_byte_ms && now - st.last_byte_ms < STATUS_WINDOW_MS) return BS_AMBER;
  return BS_GREY;
}

static lv_obj_t* make_label(lv_obj_t* parent, const lv_font_t* font, lv_color_t color, const char* txt) {
  lv_obj_t* l = lv_label_create(parent);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, color, 0);
  lv_label_set_text(l, txt);
  return l;
}

static lv_obj_t* make_card(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h) {
  lv_obj_t* c = lv_obj_create(parent);
  lv_obj_set_pos(c, x, y);
  lv_obj_set_size(c, w, h);
  lv_obj_set_style_bg_color(c, COL_CARD, 0);
  lv_obj_set_style_bg_color(c, COL_PRESS, LV_STATE_PRESSED);
  lv_obj_set_style_border_width(c, 0, 0);
  lv_obj_set_style_radius(c, 18, 0);
  lv_obj_set_style_pad_all(c, 20, 0);
  lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  return c;
}

static lv_obj_t* make_dot(lv_obj_t* parent, lv_coord_t size, lv_color_t color) {
  lv_obj_t* d = lv_obj_create(parent);
  lv_obj_set_size(d, size, size);
  lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(d, 0, 0);
  lv_obj_set_style_bg_color(d, color, 0);
  lv_obj_clear_flag(d, LV_OBJ_FLAG_CLICKABLE);
  return d;
}

// ---------- sensor readings ----------
struct Reading {
  bool     t_ok, h_ok;     // value received at least once
  float    t, h;
  uint32_t t_ms, h_ms;     // last update
};

static void read_sensor(size_t i, Reading* r) {
  const THSensor& s = TH_SENSORS[i];
  uint16_t v;
  r->t_ok = rs485_reg_get(s.slave, s.fc, s.temp_reg, &v, &r->t_ms);
  r->t = r->t_ok ? (int16_t)v * s.scale : 0;
  r->h_ok = rs485_reg_get(s.slave, s.fc, s.hum_reg, &v, &r->h_ms);
  r->h = r->h_ok ? v * s.scale : 0;
}

// Average of fresh values; if none are fresh, of the last known ones (marked stale).
struct Avg { uint8_t fresh, known; float avg, mn, mx; uint32_t newest_ms; };

static void average(bool temp, uint32_t now, Avg* a) {
  float sum_f = 0, sum_k = 0, mn_f = 0, mx_f = 0, mn_k = 0, mx_k = 0;
  *a = Avg{0, 0, 0, 0, 0, 0};
  for (size_t i = 0; i < NUM_TH_SENSORS; i++) {
    Reading r;
    read_sensor(i, &r);
    const bool ok = temp ? r.t_ok : r.h_ok;
    if (!ok) continue;
    const float v = temp ? r.t : r.h;
    const uint32_t t = temp ? r.t_ms : r.h_ms;
    if (!a->known || v < mn_k) mn_k = v;
    if (!a->known || v > mx_k) mx_k = v;
    if (!a->known || t - a->newest_ms < 0x80000000u) a->newest_ms = t;
    sum_k += v;
    a->known++;
    if (now - t <= VALUE_STALE_MS) {
      if (!a->fresh || v < mn_f) mn_f = v;
      if (!a->fresh || v > mx_f) mx_f = v;
      sum_f += v;
      a->fresh++;
    }
  }
  if (a->fresh) { a->avg = sum_f / a->fresh; a->mn = mn_f; a->mx = mx_f; }
  else if (a->known) { a->avg = sum_k / a->known; a->mn = mn_k; a->mx = mx_k; }
}

// =====================================================================
//  detail screen: every sensor's own values
// =====================================================================
static void update_detail(uint32_t now) {
  char buf[32];
  for (size_t i = 0; i < NUM_TH_SENSORS; i++) {
    const THSensor& s = TH_SENSORS[i];
    Reading r;
    read_sensor(i, &r);
    const bool ok = s_detail_temp ? r.t_ok : r.h_ok;
    const uint32_t t = s_detail_temp ? r.t_ms : r.h_ms;
    const uint16_t row = (uint16_t)(i + 1);
    lv_table_set_cell_value(s_detail_table, row, 0, s.name);
    if (ok) {
      fmt_fixed(buf, sizeof(buf), s_detail_temp ? r.t : r.h, s.decimals);
      strcat(buf, s_detail_temp ? " " DEG_C : " %");
    } else {
      snprintf(buf, sizeof(buf), "--");
    }
    lv_table_set_cell_value(s_detail_table, row, 1, buf);
    if (!ok) {
      snprintf(buf, sizeof(buf), "Waiting");
    } else {
      char d[20];
      fmt_duration(d, sizeof(d), now - t);
      snprintf(buf, sizeof(buf), now - t > VALUE_STALE_MS ? "Old (%s ago)" : "%s ago", d);
    }
    lv_table_set_cell_value(s_detail_table, row, 2, buf);
  }
}

static void detail_back_cb(lv_event_t*) { lv_obj_add_flag(s_detail, LV_OBJ_FLAG_HIDDEN); }

static void tile_clicked_cb(lv_event_t* e) {
  s_detail_temp = lv_event_get_user_data(e) != nullptr;
  lv_label_set_text(s_detail_title, s_detail_temp ? "Temperature" : "Humidity");
  lv_table_set_cell_value(s_detail_table, 0, 1, s_detail_temp ? "Temperature" : "Humidity");
  update_detail(millis());
  lv_obj_clear_flag(s_detail, LV_OBJ_FLAG_HIDDEN);
}

static void build_detail() {
  s_detail = lv_obj_create(lv_layer_top());
  lv_obj_set_size(s_detail, 1024, 600);
  lv_obj_set_pos(s_detail, 0, 0);
  lv_obj_set_style_bg_color(s_detail, COL_BG, 0);
  lv_obj_set_style_bg_opa(s_detail, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(s_detail, 0, 0);
  lv_obj_set_style_radius(s_detail, 0, 0);
  lv_obj_set_style_pad_all(s_detail, 16, 0);
  lv_obj_clear_flag(s_detail, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* back = lv_btn_create(s_detail);
  lv_obj_set_size(back, 200, BTN_H);
  lv_obj_align(back, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_add_event_cb(back, detail_back_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* l = make_label(back, FONT_L, lv_color_white(), LV_SYMBOL_LEFT "  Back");
  lv_obj_center(l);
  s_detail_title = make_label(s_detail, FONT_XL, COL_TEXT, "Temperature");
  lv_obj_align(s_detail_title, LV_ALIGN_TOP_LEFT, 230, 16);

  s_detail_table = lv_table_create(s_detail);
  lv_table_set_col_cnt(s_detail_table, 3);
  lv_table_set_row_cnt(s_detail_table, (uint16_t)(NUM_TH_SENSORS + 1));
  static const char* const heads[] = {"Sensor", "Temperature", "Updated"};
  static const lv_coord_t widths[] = {380, 300, 312};
  for (uint16_t c = 0; c < 3; c++) {
    lv_table_set_col_width(s_detail_table, c, widths[c]);
    lv_table_set_cell_value(s_detail_table, 0, c, heads[c]);
  }
  lv_obj_set_style_text_font(s_detail_table, FONT_L, LV_PART_ITEMS);
  lv_obj_set_style_pad_ver(s_detail_table, 16, LV_PART_ITEMS);
  lv_obj_set_style_bg_color(s_detail_table, COL_CARD, LV_PART_ITEMS);
  lv_obj_set_style_text_color(s_detail_table, COL_TEXT, LV_PART_ITEMS);
  lv_obj_set_style_border_color(s_detail_table, COL_PRESS, LV_PART_ITEMS);
  lv_obj_set_style_bg_color(s_detail_table, COL_CARD, 0);
  lv_obj_set_style_border_width(s_detail_table, 0, 0);
  lv_obj_set_size(s_detail_table, 992, 470);
  lv_obj_align(s_detail_table, LV_ALIGN_TOP_LEFT, 0, BTN_H + 20);

  lv_obj_add_flag(s_detail, LV_OBJ_FLAG_HIDDEN);
}

// =====================================================================
//  HOME screen
// =====================================================================
// ---------- "Resetting..." spinner, then a short "Counts reset" tick ----------
static lv_obj_t* s_busy;
static lv_obj_t* s_busy_spin;
static lv_obj_t* s_busy_lbl;

static void busy_close_cb(lv_timer_t*) {
  if (s_busy) { lv_obj_del(s_busy); s_busy = nullptr; }
}

static void busy_done_cb(lv_timer_t*) {
  if (!s_busy) return;
  lv_obj_add_flag(s_busy_spin, LV_OBJ_FLAG_HIDDEN);
  lv_label_set_text(s_busy_lbl, LV_SYMBOL_OK "  Counts reset");
  lv_obj_set_style_text_color(s_busy_lbl, COL_OK, 0);
  lv_obj_align(s_busy_lbl, LV_ALIGN_CENTER, 0, 0);
  lv_timer_set_repeat_count(lv_timer_create(busy_close_cb, 900, nullptr), 1);
}

static void show_reset_animation() {
  if (s_busy) return;
  s_busy = lv_obj_create(lv_layer_top());                 // dims the screen and blocks touches
  lv_obj_set_size(s_busy, 1024, 600);
  lv_obj_set_pos(s_busy, 0, 0);
  lv_obj_set_style_bg_color(s_busy, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(s_busy, LV_OPA_60, 0);
  lv_obj_set_style_border_width(s_busy, 0, 0);
  lv_obj_set_style_radius(s_busy, 0, 0);
  lv_obj_clear_flag(s_busy, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* card = make_card(s_busy, 0, 0, 380, 230);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(card);
  s_busy_spin = lv_spinner_create(card, 1000, 60);
  lv_obj_set_size(s_busy_spin, 100, 100);
  lv_obj_align(s_busy_spin, LV_ALIGN_TOP_MID, 0, 0);
  s_busy_lbl = make_label(card, FONT_L, COL_TEXT, "Resetting...");
  lv_obj_align(s_busy_lbl, LV_ALIGN_BOTTOM_MID, 0, 0);

  lv_timer_set_repeat_count(lv_timer_create(busy_done_cb, 1200, nullptr), 1);
}

static void reset_confirm_cb(lv_event_t* e) {
  lv_obj_t* mbox = lv_event_get_current_target(e);
  const char* txt = lv_msgbox_get_active_btn_text(mbox);
  if (txt && strcmp(txt, "Reset") == 0) {
    door_reset_counts();
    show_reset_animation();
  }
  lv_msgbox_close_async(mbox);   // deleting it inside its own event is unsafe
}

static void silence_cb(lv_event_t*) {
  door_silence_alarm();
  lv_obj_add_flag(s_silence_btn, LV_OBJ_FLAG_HIDDEN);
}

static void reset_cb(lv_event_t*) {
  static const char* btns[] = {"Reset", "Cancel", ""};
  lv_obj_t* m = lv_msgbox_create(nullptr, "Reset door counts?", "Opened and closed counts go back to 0.", btns, false);
  lv_obj_set_width(m, 620);
  lv_obj_set_style_pad_all(m, 24, 0);
  lv_obj_set_style_text_font(m, FONT_L, 0);
  lv_obj_t* bm = lv_msgbox_get_btns(m);           // Reset and Cancel, far apart so they are hard to mix up
  lv_obj_set_size(bm, LV_PCT(100), BTN_H);
  lv_obj_set_style_pad_column(bm, 140, 0);
  lv_obj_set_style_pad_top(bm, 20, 0);
  lv_obj_set_style_text_font(bm, FONT_L, LV_PART_ITEMS);  lv_obj_add_event_cb(m, reset_confirm_cb, LV_EVENT_VALUE_CHANGED, nullptr);
  lv_obj_center(m);
}

static void build_value_tile(lv_obj_t* scr, lv_coord_t x, bool temp, const char* title, lv_color_t accent,
                             lv_obj_t** val, lv_obj_t** sub) {
  lv_obj_t* c = make_card(scr, x, 68, 488, 250);
  lv_obj_add_event_cb(c, tile_clicked_cb, LV_EVENT_CLICKED, temp ? (void*)1 : nullptr);
  lv_obj_set_style_border_side(c, LV_BORDER_SIDE_LEFT, 0);
  lv_obj_set_style_border_width(c, 8, 0);
  lv_obj_set_style_border_color(c, accent, 0);

  lv_obj_t* l = make_label(c, FONT_L, COL_MUTED, title);
  lv_obj_align(l, LV_ALIGN_TOP_LEFT, 0, 0);
  *val = make_label(c, FONT_XL, COL_TEXT, "--");
  lv_obj_align(*val, LV_ALIGN_LEFT_MID, 0, -6);
  *sub = make_label(c, FONT_M, COL_MUTED, "Waiting for sensor data");
  lv_obj_align(*sub, LV_ALIGN_LEFT_MID, 0, 36);
  l = make_label(c, FONT_M, accent, "Tap for each sensor  " LV_SYMBOL_RIGHT);
  lv_obj_align(l, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
}

static void build_home(lv_obj_t* scr) {
  // header
  lv_obj_t* l = make_label(scr, FONT_XL, COL_TEXT, DASHBOARD_TITLE);
  lv_obj_set_pos(l, 20, 14);
  s_bus_lbl = make_label(scr, FONT_M, COL_MUTED, "");
  lv_obj_align(s_bus_lbl, LV_ALIGN_TOP_RIGHT, -20, 22);
  s_bus_dot = make_dot(scr, 16, COL_IDLE);
  lv_obj_align_to(s_bus_dot, s_bus_lbl, LV_ALIGN_OUT_LEFT_MID, -10, 0);

  build_value_tile(scr, 16, true, "AVERAGE TEMPERATURE", COL_TEMP, &s_temp_val, &s_temp_sub);
  build_value_tile(scr, 520, false, "AVERAGE HUMIDITY", COL_HUM, &s_hum_val, &s_hum_sub);

  // door
  lv_obj_t* d = make_card(scr, 16, 334, 992, 200);
  lv_obj_clear_flag(d, LV_OBJ_FLAG_CLICKABLE);
  l = make_label(d, FONT_L, COL_MUTED, "DOOR");
  lv_obj_align(l, LV_ALIGN_TOP_LEFT, 0, 0);
  s_door_dot = make_dot(d, 28, COL_IDLE);
  lv_obj_align(s_door_dot, LV_ALIGN_LEFT_MID, 0, 6);
  s_door_state = make_label(d, FONT_XL, COL_TEXT, "--");
  lv_obj_align(s_door_state, LV_ALIGN_LEFT_MID, 42, 6);
  s_door_since = make_label(d, FONT_M, COL_MUTED, "");
  lv_obj_align(s_door_since, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  l = make_label(d, FONT_L, COL_MUTED, "Opened");
  lv_obj_align(l, LV_ALIGN_TOP_LEFT, 340, 30);
  s_door_opens = make_label(d, FONT_XL, COL_TEXT, "0");
  lv_obj_align(s_door_opens, LV_ALIGN_TOP_LEFT, 340, 76);
  l = make_label(d, FONT_L, COL_MUTED, "Closed");
  lv_obj_align(l, LV_ALIGN_TOP_LEFT, 540, 30);
  s_door_closes = make_label(d, FONT_XL, COL_TEXT, "0");
  lv_obj_align(s_door_closes, LV_ALIGN_TOP_LEFT, 540, 76);

  lv_obj_t* b = lv_btn_create(d);
  lv_obj_set_size(b, 230, BTN_H);
  lv_obj_align(b, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(b, COL_PRESS, 0);
  lv_obj_add_event_cb(b, reset_cb, LV_EVENT_CLICKED, nullptr);
  l = make_label(b, FONT_L, COL_TEXT, "Reset counts");
  lv_obj_center(l);

  s_silence_btn = lv_btn_create(d);
  lv_obj_set_size(s_silence_btn, 230, BTN_H);
  lv_obj_align(s_silence_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(s_silence_btn, COL_ALARM, 0);
  lv_obj_add_event_cb(s_silence_btn, silence_cb, LV_EVENT_CLICKED, nullptr);
  l = make_label(s_silence_btn, FONT_L, lv_color_white(), LV_SYMBOL_MUTE "  Silence");
  lv_obj_center(l);
  lv_obj_add_flag(s_silence_btn, LV_OBJ_FLAG_HIDDEN);

  // brand names along the bottom edge
  l = make_label(scr, FONT_L, COL_TEXT, "1ACCORD");
  lv_obj_align(l, LV_ALIGN_BOTTOM_LEFT, 24, -18);
  l = make_label(scr, FONT_L, COL_TEXT, "COOL CROP");
  lv_obj_align(l, LV_ALIGN_BOTTOM_RIGHT, -24, -18);
}

static void update_value_tile(bool temp, uint32_t now, lv_obj_t* val, lv_obj_t* sub) {
  Avg a;
  average(temp, now, &a);
  const char* unit = temp ? " " DEG_C : " %";
  if (!a.known) {
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_color(val, COL_MUTED, 0);
    lv_label_set_text(sub, "Waiting for sensor data");
    return;
  }
  char v[24], mn[16], mx[16];
  fmt_fixed(v, sizeof(v), a.avg, 1);
  strcat(v, unit);
  lv_label_set_text(val, v);
  lv_obj_set_style_text_color(val, a.fresh ? COL_TEXT : COL_MUTED, 0);
  fmt_fixed(mn, sizeof(mn), a.mn, 1);
  fmt_fixed(mx, sizeof(mx), a.mx, 1);
  if (a.fresh) {
    snprintf(s_txt, sizeof(s_txt), "%u of %u sensors   |   low %s   high %s", a.fresh, (unsigned)NUM_TH_SENSORS, mn, mx);
  } else {
    char d[20];
    fmt_duration(d, sizeof(d), now - a.newest_ms);
    snprintf(s_txt, sizeof(s_txt), "Not updating - last reading %s ago", d);
  }
  lv_label_set_text(sub, s_txt);
}

static void update_home(uint32_t now) {
  // bus status
  const BusState bs = bus_state(s_st, now);
  lv_label_set_text(s_bus_lbl, bs == BS_GREEN ? "Sensors online"
                             : bs == BS_AMBER ? "Sensor connection problem"
                                              : "Waiting for sensors");
  lv_obj_set_style_bg_color(s_bus_dot, bs == BS_GREEN ? COL_OK : bs == BS_AMBER ? COL_WARN : COL_IDLE, 0);
  lv_obj_align_to(s_bus_dot, s_bus_lbl, LV_ALIGN_OUT_LEFT_MID, -10, 0);

  update_value_tile(true, now, s_temp_val, s_temp_sub);
  update_value_tile(false, now, s_hum_val, s_hum_sub);

  // door (red + blinking dot once it has been open too long)
  const lv_color_t door_col = s_door.alarm ? COL_ALARM : s_door.open ? COL_WARN : COL_OK;
  lv_label_set_text(s_door_state, s_door.alarm ? "OPEN TOO LONG" : s_door.open ? "OPEN" : "CLOSED");
  lv_obj_set_style_text_color(s_door_state, door_col, 0);
  lv_obj_set_style_bg_color(s_door_dot, door_col, 0);
  lv_obj_set_style_bg_opa(s_door_dot, s_door.alarm && (now / 500) % 2 ? LV_OPA_20 : LV_OPA_COVER, 0);
  char d[24];
  fmt_duration(d, sizeof(d), now - s_door.since_ms);
  snprintf(s_txt, sizeof(s_txt), "%s for %s", s_door.open ? "Open" : "Closed", d);
  lv_label_set_text(s_door_since, s_txt);
  if (s_door.alarm && !s_door.silenced && buzzer_present()) lv_obj_clear_flag(s_silence_btn, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_add_flag(s_silence_btn, LV_OBJ_FLAG_HIDDEN);
  snprintf(s_txt, sizeof(s_txt), "%lu", (unsigned long)s_door.opens);
  lv_label_set_text(s_door_opens, s_txt);
  snprintf(s_txt, sizeof(s_txt), "%lu", (unsigned long)s_door.closes);
  lv_label_set_text(s_door_closes, s_txt);
}

// =====================================================================
//  timer + create
// =====================================================================
static void ui_tick(lv_timer_t*) {
  static uint32_t last_full_redraw, last_saves;
  const uint32_t now = millis();
  rs485_get_stats(&s_st);
  door_get(&s_door);
  if (lv_obj_has_flag(s_detail, LV_OBJ_FLAG_HIDDEN)) update_home(now);
  else update_detail(now);

  // Self-heal for ghosting/flicker: the panel uses two frame buffers and LVGL only
  // copies changed areas between them. If a copy is lost (flash write, electrical
  // noise) the buffers differ and the screen alternates between old and new images.
  // A full redraw rewrites both buffers. Done periodically and right after every
  // flash write of the door counts.
  if (now - last_full_redraw >= SCREEN_REFRESH_MS || s_door.saves != last_saves) {
    last_full_redraw = now;
    last_saves = s_door.saves;
    lv_obj_invalidate(lv_scr_act());
  }
}

// ---------- start-up splash: names, spinner and a loading bar, then HOME ----------
static lv_obj_t* s_splash;

static void splash_close_cb(lv_timer_t*) {
  if (s_splash) { lv_obj_del(s_splash); s_splash = nullptr; }
  lv_obj_invalidate(lv_scr_act());
}

static void show_splash() {
  s_splash = lv_obj_create(lv_layer_top());
  lv_obj_set_size(s_splash, 1024, 600);
  lv_obj_set_pos(s_splash, 0, 0);
  lv_obj_set_style_bg_color(s_splash, COL_BG, 0);
  lv_obj_set_style_bg_opa(s_splash, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(s_splash, 0, 0);
  lv_obj_set_style_radius(s_splash, 0, 0);
  lv_obj_clear_flag(s_splash, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* l = make_label(s_splash, FONT_XL, COL_TEXT, "1ACCORD");
  lv_obj_align(l, LV_ALIGN_CENTER, 0, -150);
  l = make_label(s_splash, FONT_L, COL_HUM, "COOL CROP");
  lv_obj_align(l, LV_ALIGN_CENTER, 0, -105);

  lv_obj_t* spin = lv_spinner_create(s_splash, 1000, 60);
  lv_obj_set_size(spin, 110, 110);
  lv_obj_align(spin, LV_ALIGN_CENTER, 0, 10);

  lv_obj_t* bar = lv_bar_create(s_splash);
  lv_obj_set_size(bar, 420, 14);
  lv_obj_align(bar, LV_ALIGN_CENTER, 0, 110);
  lv_bar_set_range(bar, 0, 100);
  lv_obj_set_style_anim_time(bar, 2200, 0);
  lv_bar_set_value(bar, 100, LV_ANIM_ON);          // fills smoothly over 2.2 s

  l = make_label(s_splash, FONT_M, COL_MUTED, "Starting...");
  lv_obj_align(l, LV_ALIGN_CENTER, 0, 145);

  lv_timer_set_repeat_count(lv_timer_create(splash_close_cb, 2600, nullptr), 1);
}

void ui_create() {
  lv_disp_t* disp = lv_disp_get_default();
  lv_theme_t* th = lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE),
                                         lv_palette_main(LV_PALETTE_CYAN), true, FONT_M);
  lv_disp_set_theme(disp, th);

  lv_obj_t* scr = lv_scr_act();
  lv_obj_clean(scr);
  lv_obj_set_style_bg_color(scr, COL_BG, 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // backlight level saved by earlier firmware (default full brightness)
  Preferences p;
  p.begin("rs485sniff", false);
  uint8_t bl = p.getUChar("bl", 100);
  p.end();
  if (bl < 10 || bl > 100) bl = 100;
  IO_EXTENSION_Pwm_Output((uint8_t)(100 - bl));      // Waveshare PWM is inverted: 0 = full brightness

  build_home(scr);
  build_detail();
  show_splash();

  lv_timer_create(ui_tick, 250, nullptr);
  ui_tick(nullptr);

  lv_mem_monitor_t mon;
  lv_mem_monitor(&mon);
  Serial.printf("UI created. LVGL heap used %u%% (%lu bytes free)\r\n", mon.used_pct, (unsigned long)mon.free_size);
}
