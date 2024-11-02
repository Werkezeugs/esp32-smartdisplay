#include <esp32_smartdisplay.h>
#include <esp_lcd_panel_ops.h>

#ifdef BOARD_HAS_TOUCH
#include <esp_lcd_touch.h>
#endif

// Defines for adaptive brightness adjustment
#define BRIGHTNESS_SMOOTHING_MEASUREMENTS 100
#define BRIGHTNESS_DARK_ZONE 250

// Functions to be defined in the tft/touch driver
extern lv_display_t *lvgl_lcd_init();
extern lv_indev_t *lvgl_touch_init();

lv_display_t *display;

#ifdef BOARD_HAS_TOUCH
lv_indev_t *indev;
touch_calibration_data_t touch_calibration_data;
void (*driver_touch_read_cb)(lv_indev_t *indev, lv_indev_data_t *data);
#endif

void lvgl_display_resolution_changed_callback(lv_event_t *drv);

lv_timer_t *update_brightness_timer;

#ifdef LV_USE_LOG
void lvgl_log(lv_log_level_t level, const char *buf)
{
  switch (level)
  {
  case LV_LOG_LEVEL_TRACE:
    log_printf("%s", buf);
    break;
  case LV_LOG_LEVEL_INFO:
    log_i("%s", buf);
    break;
  case LV_LOG_LEVEL_WARN:
    log_w("%s", buf);
    break;
  case LV_LOG_LEVEL_ERROR:
    log_e("%s", buf);
    break;
  }
}
#endif

// Set backlight intensity
void smartdisplay_lcd_set_backlight(float duty)
{
  log_v("duty:%2f", duty);

  if (duty > 1.0f)
    duty = 1.0f;
  if (duty < 0.0f)
    duty = 0.0f;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(GPIO_BCKL, duty * PWM_MAX_BCKL);
#else
  ledcWrite(PWM_CHANNEL_BCKL, duty * PWM_MAX_BCKL);
#endif
}

#ifdef BOARD_HAS_CDS
// Read CdS sensor and return a value for the screen brightness
float smartdisplay_lcd_adaptive_brightness_cds()
{
  static float avgCds;
  // Read CdS sensor
  uint16_t sensorValue = analogRead(CDS);
  // Approximation of moving average for the sensor
  avgCds -= avgCds / BRIGHTNESS_SMOOTHING_MEASUREMENTS;
  avgCds += sensorValue / BRIGHTNESS_SMOOTHING_MEASUREMENTS;
  // Section of interest is 0 (full light) until ~500 (darkish)
  int16_t lightValue = BRIGHTNESS_DARK_ZONE - avgCds;
  if (lightValue < 0)
    lightValue = 0;
  // Set fixed percentage and variable based on CdS sensor
  return 0.01 + (0.99 / BRIGHTNESS_DARK_ZONE) * lightValue;
}
#endif

void adaptive_brightness(lv_timer_t *timer)
{
  log_v("timer:0x%08x", timer);

  const smartdisplay_lcd_adaptive_brightness_cb_t callback = timer->user_data;
  smartdisplay_lcd_set_backlight(callback());
}

void smartdisplay_lcd_set_brightness_cb(smartdisplay_lcd_adaptive_brightness_cb_t cb, uint interval)
{
  log_v("adaptive_brightness_cb:0x%08x, interval:%u", cb, interval);

  // Delete current timer if any
  if (update_brightness_timer)
    lv_timer_del(update_brightness_timer);

  // Use callback for intensity or 50% default
  if (cb && interval > 0)
    update_brightness_timer = lv_timer_create(adaptive_brightness, interval, cb);
  else
    smartdisplay_lcd_set_backlight(0.5f);
}

#ifdef BOARD_HAS_RGB_LED
void smartdisplay_led_set_rgb(bool r, bool g, bool b)
{
  log_d("R:%d, G:%d, B:%d", r, b, b);

  digitalWrite(RGB_LED_R, !r);
  digitalWrite(RGB_LED_G, !g);
  digitalWrite(RGB_LED_B, !b);
}
#endif

#ifdef BOARD_HAS_TOUCH
// See: https://www.maximintegrated.com/en/design/technical-documents/app-notes/5/5296.html
void lvgl_touch_calibration_transform(lv_indev_t *indev, lv_indev_data_t *data)
{
  log_v("indev:0x%08x, data:0x%08x", indev, data);

  // Call low level read from the driver
  driver_touch_read_cb(indev, data);
  // Check if transformation is required
  if (touch_calibration_data.valid && data->state == LV_INDEV_STATE_PRESSED)
  {
    log_d("lvgl_touch_calibration_transform: transformation applied");
    lv_point_t pt = {
        .x = roundf(data->point.x * touch_calibration_data.alphaX + data->point.y * touch_calibration_data.betaX + touch_calibration_data.deltaX),
        .y = roundf(data->point.x * touch_calibration_data.alphaY + data->point.y * touch_calibration_data.betaY + touch_calibration_data.deltaY)};
    log_d("Calibrate point (%d, %d) => (%d, %d)", data->point.x, data->point.y, pt.x, pt.y);
    data->point = (lv_point_t){pt.x, pt.y};
  }
}

touch_calibration_data_t smartdisplay_compute_touch_calibration(const lv_point_t screen[3], const lv_point_t touch[3])
{
  log_v("screen:0x%08x, touch:0x%08x", screen, touch);
  const float delta = ((touch[0].x - touch[2].x) * (touch[1].y - touch[2].y)) - ((touch[1].x - touch[2].x) * (touch[0].y - touch[2].y));
  touch_calibration_data_t touch_calibration_data = {
      .valid = true,
      .alphaX = (((screen[0].x - screen[2].x) * (touch[1].y - touch[2].y)) - ((screen[1].x - screen[2].x) * (touch[0].y - touch[2].y))) / delta,
      .betaX = (((touch[0].x - touch[2].x) * (screen[1].x - screen[2].x)) - ((touch[1].x - touch[2].x) * (screen[0].x - screen[2].x))) / delta,
      .deltaX = ((screen[0].x * ((touch[1].x * touch[2].y) - (touch[2].x * touch[1].y))) - (screen[1].x * ((touch[0].x * touch[2].y) - (touch[2].x * touch[0].y))) + (screen[2].x * ((touch[0].x * touch[1].y) - (touch[1].x * touch[0].y)))) / delta,
      .alphaY = (((screen[0].y - screen[2].y) * (touch[1].y - touch[2].y)) - ((screen[1].y - screen[2].y) * (touch[0].y - touch[2].y))) / delta,
      .betaY = (((touch[0].x - touch[2].x) * (screen[1].y - screen[2].y)) - ((touch[1].x - touch[2].x) * (screen[0].y - screen[2].y))) / delta,
      .deltaY = ((screen[0].y * (touch[1].x * touch[2].y - touch[2].x * touch[1].y)) - (screen[1].y * (touch[0].x * touch[2].y - touch[2].x * touch[0].y)) + (screen[2].y * (touch[0].x * touch[1].y - touch[1].x * touch[0].y))) / delta,
  };

  log_d("alphaX: %f, betaX: %f, deltaX: %f, alphaY: %f, betaY: %f, deltaY: %f", touch_calibration_data.alphaX, touch_calibration_data.betaX, touch_calibration_data.deltaX, touch_calibration_data.alphaY, touch_calibration_data.betaY, touch_calibration_data.deltaY);
  return touch_calibration_data;
};
#endif

void smartdisplay_init()
{
  log_d("smartdisplay_init");
#ifdef BOARD_HAS_RGB_LED
  // Setup RGB LED.  High is off
  pinMode(RGB_LED_R, OUTPUT);
  pinMode(RGB_LED_G, OUTPUT);
  pinMode(RGB_LED_B, OUTPUT);
  smartdisplay_led_set_rgb(false, false, false);
#endif

#ifdef BOARD_HAS_CDS
  // CDS Light sensor
  pinMode(CDS, INPUT);
  analogSetAttenuation(ADC_0db); // 0dB(1.0x) 0~800mV
#endif

#ifdef BOARD_HAS_SPEAK
  // Speaker
  // Note: tone function uses PWM channel 0
  pinMode(SPEAK, OUTPUT);
#endif

#if LV_USE_LOG
  lv_log_register_print_cb(lvgl_log);
#endif

  lv_init();
  // Setup backlight
  pinMode(GPIO_BCKL, OUTPUT);
  digitalWrite(GPIO_BCKL, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(GPIO_BCKL, PWM_FREQ_BCKL, PWM_BITS_BCKL);
#else
  ledcSetup(PWM_CHANNEL_BCKL, PWM_FREQ_BCKL, PWM_BITS_BCKL);
  ledcAttachPin(GPIO_BCKL, PWM_CHANNEL_BCKL);
#endif
  // Setup TFT display
  display = lvgl_lcd_init();
  // Register callback for hardware rotation
  if (!display->rotation)
    lv_display_add_event_cb(display, lvgl_display_resolution_changed_callback, LV_EVENT_RESOLUTION_CHANGED, NULL);

  //  Clear screen
  lv_obj_clean(lv_scr_act());
  // Turn backlight on (50%)
  smartdisplay_lcd_set_backlight(0.5f);

// If there is a touch controller defined
#ifdef BOARD_HAS_TOUCH
  // Setup touch
  indev = lvgl_touch_init();
  indev->disp = display;
  // Intercept callback
  driver_touch_read_cb = indev->read_cb;
  indev->read_cb = lvgl_touch_calibration_transform;
  lv_indev_enable(indev, true);
#endif
}

// Called when driver resolution is updated (including rotation)
// Top of the display is top left when connector is at the bottom
// The rotation values are relative to how you would rotate the physical display in the clockwise direction.
// Thus, LV_DISPLAY_ROTATION_90 means you rotate the hardware 90 degrees clockwise, and the display rotates 90 degrees counterclockwise to compensate.
void lvgl_display_resolution_changed_callback(lv_event_t *event)
{
  const esp_lcd_panel_handle_t panel_handle = display->user_data;
  switch (display->rotation)
  {
  case LV_DISPLAY_ROTATION_0:
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, DISPLAY_SWAP_XY));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
    break;
  case LV_DISPLAY_ROTATION_90:
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, !DISPLAY_SWAP_XY));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, DISPLAY_MIRROR_X, !DISPLAY_MIRROR_Y));
    break;
  case LV_DISPLAY_ROTATION_180:
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, DISPLAY_SWAP_XY));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, !DISPLAY_MIRROR_X, !DISPLAY_MIRROR_Y));
    break;
  case LV_DISPLAY_ROTATION_270:
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, !DISPLAY_SWAP_XY));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, !DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
    break;
  }
}

lv_display_t *lvgl_lcd_init()
{
    lv_display_t *display = lv_display_create(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    log_v("display:0x%08x", display);
    //  Create drawBuffer
    lv_color_format_t cf = lv_display_get_color_format(display);
    uint32_t px_size = lv_color_format_get_size(cf);
    uint32_t drawBufferSize = px_size * LVGL_BUFFER_PIXELS;
    void *drawBuffer = heap_caps_malloc(drawBufferSize, LVGL_BUFFER_MALLOC_FLAGS);
    lv_display_set_buffers(display, drawBuffer, NULL, drawBufferSize, LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Hardware rotation is not supported
    display->rotation = LV_DISPLAY_ROTATION_0;

    // Create direct_io panel handle
    const esp_lcd_rgb_panel_config_t rgb_panel_config = {
        .clk_src = ST7262_PANEL_CONFIG_CLK_SRC,
        .timings = {
            .pclk_hz = ST7262_PANEL_CONFIG_TIMINGS_PCLK_HZ,
            .h_res = ST7262_PANEL_CONFIG_TIMINGS_H_RES,
            .v_res = ST7262_PANEL_CONFIG_TIMINGS_V_RES,
            .hsync_pulse_width = ST7262_PANEL_CONFIG_TIMINGS_HSYNC_PULSE_WIDTH,
            .hsync_back_porch = ST7262_PANEL_CONFIG_TIMINGS_HSYNC_BACK_PORCH,
            .hsync_front_porch = ST7262_PANEL_CONFIG_TIMINGS_HSYNC_FRONT_PORCH,
            .vsync_pulse_width = ST7262_PANEL_CONFIG_TIMINGS_VSYNC_PULSE_WIDTH,
            .vsync_back_porch = ST7262_PANEL_CONFIG_TIMINGS_VSYNC_BACK_PORCH,
            .vsync_front_porch = ST7262_PANEL_CONFIG_TIMINGS_VSYNC_FRONT_PORCH,
            .flags = {
                .hsync_idle_low = ST7262_PANEL_CONFIG_TIMINGS_FLAGS_HSYNC_IDLE_LOW,
                .vsync_idle_low = ST7262_PANEL_CONFIG_TIMINGS_FLAGS_VSYNC_IDLE_LOW,
                .de_idle_high = ST7262_PANEL_CONFIG_TIMINGS_FLAGS_DE_IDLE_HIGH,
                .pclk_active_neg = ST7262_PANEL_CONFIG_TIMINGS_FLAGS_PCLK_ACTIVE_NEG,
                .pclk_idle_high = ST7262_PANEL_CONFIG_TIMINGS_FLAGS_PCLK_IDLE_HIGH}},
        .data_width = ST7262_PANEL_CONFIG_DATA_WIDTH,
        .sram_trans_align = ST7262_PANEL_CONFIG_SRAM_TRANS_ALIGN,
        .psram_trans_align = ST7262_PANEL_CONFIG_PSRAM_TRANS_ALIGN,
        .hsync_gpio_num = ST7262_PANEL_CONFIG_HSYNC_GPIO_NUM,
        .vsync_gpio_num = ST7262_PANEL_CONFIG_VSYNC_GPIO_NUM,
        .de_gpio_num = ST7262_PANEL_CONFIG_DE_GPIO_NUM,
        .pclk_gpio_num = ST7262_PANEL_CONFIG_PCLK_GPIO_NUM,
        // LV_COLOR_16_SWAP is handled by mapping of the data
        .data_gpio_nums = {ST7262_PANEL_CONFIG_DATA_GPIO_R0, ST7262_PANEL_CONFIG_DATA_GPIO_R1, ST7262_PANEL_CONFIG_DATA_GPIO_R2, ST7262_PANEL_CONFIG_DATA_GPIO_R3, ST7262_PANEL_CONFIG_DATA_GPIO_R4, ST7262_PANEL_CONFIG_DATA_GPIO_G0, ST7262_PANEL_CONFIG_DATA_GPIO_G1, ST7262_PANEL_CONFIG_DATA_GPIO_G2, ST7262_PANEL_CONFIG_DATA_GPIO_G3, ST7262_PANEL_CONFIG_DATA_GPIO_G4, ST7262_PANEL_CONFIG_DATA_GPIO_G5, ST7262_PANEL_CONFIG_DATA_GPIO_B0, ST7262_PANEL_CONFIG_DATA_GPIO_B1, ST7262_PANEL_CONFIG_DATA_GPIO_B2, ST7262_PANEL_CONFIG_DATA_GPIO_B3, ST7262_PANEL_CONFIG_DATA_GPIO_B4},
        .disp_gpio_num = ST7262_PANEL_CONFIG_DISP_GPIO_NUM,
        .on_frame_trans_done = direct_io_frame_trans_done,
        .user_ctx = display,
        .flags = {.disp_active_low = ST7262_PANEL_CONFIG_FLAGS_DISP_ACTIVE_LOW, .relax_on_idle = ST7262_PANEL_CONFIG_FLAGS_RELAX_ON_IDLE, .fb_in_psram = ST7262_PANEL_CONFIG_FLAGS_FB_IN_PSRAM}};
    log_d("rgb_panel_config: clk_src:%d, timings:{pclk_hz:%d, h_res:%d, v_res:%d, hsync_pulse_width:%d, hsync_back_porch:%d, hsync_front_porch:%d, vsync_pulse_width:%d, vsync_back_porch:%d, vsync_front_porch:%d, flags:{hsync_idle_low:%d, vsync_idle_low:%d, de_idle_high:%d, pclk_active_neg:%d, pclk_idle_high:%d}}, data_width:%d, sram_trans_align:%d, psram_trans_align:%d, hsync_gpio_num:%d, vsync_gpio_num:%d, de_gpio_num:%d, pclk_gpio_num:%d, data_gpio_nums:[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,], disp_gpio_num:%d, on_frame_trans_done:0x%08x, user_ctx:0x%08x, flags:{disp_active_low:%d, relax_on_idle:%d, fb_in_psram:%d}", rgb_panel_config.clk_src, rgb_panel_config.timings.pclk_hz, rgb_panel_config.timings.h_res, rgb_panel_config.timings.v_res, rgb_panel_config.timings.hsync_pulse_width, rgb_panel_config.timings.hsync_back_porch, rgb_panel_config.timings.hsync_front_porch, rgb_panel_config.timings.vsync_pulse_width, rgb_panel_config.timings.vsync_back_porch, rgb_panel_config.timings.vsync_front_porch, rgb_panel_config.timings.flags.hsync_idle_low, rgb_panel_config.timings.flags.vsync_idle_low, rgb_panel_config.timings.flags.de_idle_high, rgb_panel_config.timings.flags.pclk_active_neg, rgb_panel_config.timings.flags.pclk_idle_high, rgb_panel_config.data_width, rgb_panel_config.sram_trans_align, rgb_panel_config.psram_trans_align, rgb_panel_config.hsync_gpio_num, rgb_panel_config.vsync_gpio_num, rgb_panel_config.de_gpio_num, rgb_panel_config.pclk_gpio_num, rgb_panel_config.data_gpio_nums[0], rgb_panel_config.data_gpio_nums[1], rgb_panel_config.data_gpio_nums[2], rgb_panel_config.data_gpio_nums[3], rgb_panel_config.data_gpio_nums[4], rgb_panel_config.data_gpio_nums[5], rgb_panel_config.data_gpio_nums[6], rgb_panel_config.data_gpio_nums[7], rgb_panel_config.data_gpio_nums[8], rgb_panel_config.data_gpio_nums[9], rgb_panel_config.data_gpio_nums[10], rgb_panel_config.data_gpio_nums[11], rgb_panel_config.data_gpio_nums[12], rgb_panel_config.data_gpio_nums[13], rgb_panel_config.data_gpio_nums[14], rgb_panel_config.data_gpio_nums[15], rgb_panel_config.disp_gpio_num, rgb_panel_config.on_frame_trans_done, rgb_panel_config.user_ctx, rgb_panel_config.flags.disp_active_low, rgb_panel_config.flags.relax_on_idle, rgb_panel_config.flags.fb_in_psram);
    log_d("refresh rate: %d Hz", (ST7262_PANEL_CONFIG_TIMINGS_PCLK_HZ * ST7262_PANEL_CONFIG_DATA_WIDTH) / (ST7262_PANEL_CONFIG_TIMINGS_H_RES + ST7262_PANEL_CONFIG_TIMINGS_HSYNC_PULSE_WIDTH + ST7262_PANEL_CONFIG_TIMINGS_HSYNC_BACK_PORCH + ST7262_PANEL_CONFIG_TIMINGS_HSYNC_FRONT_PORCH) / (ST7262_PANEL_CONFIG_TIMINGS_V_RES + ST7262_PANEL_CONFIG_TIMINGS_VSYNC_PULSE_WIDTH + ST7262_PANEL_CONFIG_TIMINGS_VSYNC_BACK_PORCH + ST7262_PANEL_CONFIG_TIMINGS_VSYNC_FRONT_PORCH) / SOC_LCD_RGB_DATA_WIDTH);
    esp_lcd_panel_handle_t panel_handle;
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&rgb_panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
#ifdef DISPLAY_IPS
    // If LCD is IPS invert the colors
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif
#if defined(DISPLAY_GAP_X) || defined(DISPLAY_GAP_Y)
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, DISPLAY_GAP_X, DISPLAY_GAP_Y));
#endif
    display->user_data = panel_handle;
    display->flush_cb = direct_io_lv_flush;

    return display;
}
