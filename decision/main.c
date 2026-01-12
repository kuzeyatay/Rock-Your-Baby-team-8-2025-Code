// master.c — MASTER / Decision module
// Ring UART frames: [DST][SRC][LEN][PAYLOAD...]

#include <libpynq.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h> // for log_printf
#include <unistd.h>

#define UART_CH UART0
#define MSTR 0
#define HRTBT 1
#define CRY 2
#define MTR 3

#define TIMEOUT 20 // in ms
#define MAX_PAY 5  // max payload length

// *** NEW: real-world reaction delays to match the simulator ***
#define HEARTBEAT_DELAY 14000 // ~10 s heartbeat delay (TAU)
#define CRYING_DELAY 4000     // ~2 s crying / stress delay
#define CONVERGENCE_DELAY 4000

#define VITALS_POLL_MS 100 // request HB/CRY every 100ms

// global variables for submodules (live readings)
static uint8_t last_bpm = 0;
static uint8_t last_cry = 0;

// last motor command (for HUD only)
static uint8_t g_amp = 0;
static uint8_t g_freq = 0;

// global message variables that are decoded
static uint8_t g_src = 0;
static uint8_t g_len = 0;
static uint8_t g_payload[MAX_PAY];

// Global display + font
static display_t g_disp;
static FontxFile g_fx[2];
static uint8_t g_fw = 0, g_fh = 0;

// simple on-screen log area
static int g_log_x = 0;
static int g_log_y_start = 0;
static int g_log_y = 0;
static int g_log_enabled = 0;
static int g_log_y_end = 0;

// small helpers for display

static void itoa_u(unsigned v, char *out)
{
  char tmp[16];
  int n = 0;
  if (!v)
  {
    out[0] = '0';
    out[1] = 0;
    return;
  }
  while (v && n < 16)
  {
    tmp[n++] = (char)('0' + (v % 10));
    v /= 10;
  }
  for (int i = 0; i < n; i++)
    out[i] = tmp[n - 1 - i];
  out[n] = 0;
}

static inline int clampi(int v, int lo, int hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static void clear_text_line(display_t *d, int y, int h, uint16_t bg)
{
  int x1 = 0, y1 = y - h + 2, x2 = DISPLAY_WIDTH - 1, y2 = y + 2;
  x1 = clampi(x1, 0, DISPLAY_WIDTH - 1);
  x2 = clampi(x2, 0, DISPLAY_WIDTH - 1);
  y1 = clampi(y1, 0, DISPLAY_HEIGHT - 1);
  y2 = clampi(y2, 0, DISPLAY_HEIGHT - 1);
  if (x2 < x1 || y2 < y1)
    return;
  displayDrawFillRect(d, x1, y1, x2, y2, bg);
}

// safer version
static void draw_text(display_t *d, FontxFile *fx, int x, int y,
                      const char *s, uint16_t col)
{
  if (!s)
    return;

  int fw = g_fw ? (int)g_fw : 8; // fallback
  int max_chars = (DISPLAY_WIDTH - x) / fw;
  if (max_chars <= 0)
    return;

  // copy + truncate into local buffer
  char buf[128];
  int i = 0;
  for (; i < max_chars && s[i] && i < (int)sizeof(buf) - 1; i++)
    buf[i] = s[i];
  buf[i] = '\0';

  displayDrawString(d, fx, x, y, (uint8_t *)buf, col);
}

// LOGGING HELPERS
// Used by the controller to outbut stuff onto the oled screen
static void hud_log(const char *msg)
{
  if (!g_log_enabled)
    return;

  int h = g_fh ? g_fh : 16;

  // Require a valid region
  if (g_log_y_start < 0)
    return;
  if (g_log_y_end <= 0)
    return;
  if (g_log_y_end > DISPLAY_HEIGHT)
    g_log_y_end = DISPLAY_HEIGHT;

  // If region too small for even 1 line, do nothing safely
  if (g_log_y_end - g_log_y_start < h)
    return;

  // Wrap BEFORE drawing so we never draw off-screen
  if (g_log_y < g_log_y_start || g_log_y > (g_log_y_end - h))
    g_log_y = g_log_y_start;

  // Clear exactly one line inside the log region, then draw cyan text
  clear_text_line(&g_disp, g_log_y, h, RGB_BLACK);
  draw_text(&g_disp, g_fx, g_log_x, g_log_y, msg, RGB_CYAN);

  // Advance and wrap within region
  g_log_y += h;
  if (g_log_y > (g_log_y_end - h))
    g_log_y = g_log_y_start;
}

static void log_printf(const char *fmt, ...)
{
  char buf[128];

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);

  // mirror to normal stdout too
  printf("%s", buf);

  // strip trailing newlines for display
  size_t len = strlen(buf);
  while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
  {
    buf[len - 1] = '\0';
    len--;
  }

  if (len > 0)
    hud_log(buf);
}

// UART helpers

// timeouted read of a single byte
static int timeouted_byte(int ms)
{
  int waited = 0;
  while (waited < ms)
  {
    if (uart_has_data(UART_CH))
      return (int)uart_recv(UART_CH);
    sleep_msec(1);
    waited += 1;
  }
  return -1;
}

// convenience
static int receive_byte(void)
{
  return timeouted_byte(TIMEOUT);
}

// [DST][SRC][LEN][PAYLOAD]
void send_message_raw(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len)
{
  uart_send(UART_CH, dst);
  uart_send(UART_CH, src);
  uart_send(UART_CH, len);
  for (int i = 0; i < len; i++)
  {
    uart_send(UART_CH, payload[i]);
  }
}

// helper macro to infer payload length from array
#define send_message(dst, src, payload) \
  send_message_raw(dst, src, payload, (uint8_t)sizeof(payload))

// receive a message into globals g_src, g_len, g_payload
// receive_message
// UPDATED: Now non-blocking. Returns -1 immediately if no data.
static int receive_message(void)
{
  if (!uart_has_data(UART_CH))
    return -1;

  int b = receive_byte(); // DST
  if (b < 0)
    return -1;
  uint8_t dst = (uint8_t)b;

  b = receive_byte(); // SRC
  if (b < 0)
    return -1;
  uint8_t src = (uint8_t)b;

  b = receive_byte(); // LEN
  if (b < 0)
    return -1;
  uint8_t len = (uint8_t)b;

  // If not for me, drain payload so framing stays aligned.
  if (dst != MSTR)
  {
    for (int i = 0; i < len; i++)
    {
      (void)receive_byte(); // ignore; if timeout happens, we just desync less badly than leaving bytes behind
    }
    return 0;
  }

  if (len > MAX_PAY)
    len = MAX_PAY;

  for (int i = 0; i < len; i++)
  {
    b = receive_byte();
    if (b < 0)
      return -3;
    g_payload[i] = (uint8_t)b;
  }

  g_src = src;
  g_len = len;
  return (int)g_len;
}

// Ping / random / sensor / motor commands

// send a ping to a module and expect 'A' back
#define BOOT_PING_TOTAL_MS 1500 // total time to wait for module to answer
#define BOOT_PING_RETRY_MS 100  // resend 'A' every 100ms

static int boot_ping(uint8_t dst)
{
  uint8_t payload[] = {'A'};

  int waited = 0;
  int since_send = BOOT_PING_RETRY_MS; // force immediate send first iteration

  while (waited < BOOT_PING_TOTAL_MS)
  {
    if (since_send >= BOOT_PING_RETRY_MS)
    {
      send_message(dst, MSTR, payload);
      since_send = 0;
    }

    int r = receive_message();
    if (r > 0 && g_src == dst && g_len >= 1 && g_payload[0] == 'A')
      return 1;

    sleep_msec(1);
    waited += 1;
    since_send += 1;
  }
  return 0;
}

// request random value from heartbeat or crying node (for demo)
// static int request_random(uint8_t dst)
// {
//   uint8_t payload[] = {'R'};
//   send_message(dst, MSTR, payload);

//   int waited = 0;
//   while (waited < TIMEOUT)
//   {
//     int r = receive_message();
//     if (r > 0 && g_src == dst && g_len >= 2 && g_payload[0] == 'R')
//     {
//       return g_payload[1];
//     }
//     sleep_msec(1);
//     waited += 1;
//   }

//   return -1; // timeout
// }

// request heartbeat value
static void drain_my_rx(void)
{
  // Drain any already-buffered frames addressed to MSTR.
  // This prevents "one-cycle behind" by removing late replies from previous requests.
  while (uart_has_data(UART_CH))
  {
    int r = receive_message();
    if (r < 0) break; // no complete frame available right now
  }
}

static int request_heartbeat(void)
{
  // 1) Remove any stale/late replies
  drain_my_rx();

  // 2) Send request
  uint8_t payload[] = {'H'};
  send_message(HRTBT, MSTR, payload);

  // 3) Wait long enough for the heartbeat node to respond (NOT 20 ms)
  // Start with 200 ms; adjust if needed.
  const int WAIT_MS = 200;

  int waited = 0;
  while (waited < WAIT_MS)
  {
    int r = receive_message();
    if (r > 0 && g_src == HRTBT && g_len >= 2 && g_payload[0] == 'H')
      return g_payload[1];

    sleep_msec(1);
    waited += 1;
  }
  return -1;
}


// request crying value
static int request_crying(void)
{
  uint8_t payload[] = {'C'};
  send_message(CRY, MSTR, payload);
  int waited = 0;
  while (waited < TIMEOUT)
  {
    int r = receive_message();
    if (r > 0 && g_src == CRY && g_len >= 2 && g_payload[0] == 'C')
      return g_payload[1];
    sleep_msec(1);
    waited += 1;
  }
  return -1;
}

// send motor command (amp%, freq%)
static void command_motor(uint8_t amp, uint8_t freq)
{
  g_amp = amp;
  g_freq = freq;
  uint8_t payload[] = {'M', amp, freq};
  send_message(MTR, MSTR, payload);
}

// send random motor command (for demo)
// static void motor_send_random(void)
// {
//   static bool seeded = false;
//   if (!seeded)
//   {
//     srand(time(NULL));
//     seeded = true;
//   }

//   g_amp = (uint8_t)(rand() % 101);        // 0–100
//   g_freq = (uint8_t)(20 + (rand() % 81)); // 20–100

//   uint8_t pl[] = {'M', g_amp, g_freq};
//   send_message(MTR, MSTR, pl);
// }

// Controller state + logic

// A/F grid (0-4). Start at A5 F5
static int curA = 4;
static int curF = 4;

static int is_crying_activated = 0;
static int ctrl_lastBPM = -1;
static int ctrl_lastCRY = -1;
static int thresholdBPM = 10;
static int thresholdCRY = 1;

static int prevA = -1;
static int prevF = -1;

static int anchorA_mem = -1, anchorF_mem = -1;
static int triedLeftFromAnchor = 0;
static int triedUpFromAnchor = 0;

// 0 = none/initial, 1 = LEFT, 2 = UP
static int lastMoveDir = 0;

static int hit_wall = 0; // 1 if we attempted a direction but boundary blocked this cycle

// anchor map discovered so far (0 = unknown)
static int anchorMatrix[5][5] = {0};
static int anchorLevel = 0;

static int panic_mode = 0;

// timing
static double g_algo_start_ms = 0.0;
static int g_calm_reached = 0;
static int g_calm_elapsed_ms = 0;

// monotonic time in milliseconds
static double now_msec(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

// format mm:ss into out[8] (e.g., "03:17")
static void fmt_mmss(int ms, char out[8])
{
  if (ms < 0)
    ms = 0;
  int sec = ms / 1000;
  int mm = sec / 60;
  int ss = sec % 60;
  out[0] = (char)('0' + (mm / 10) % 10);
  out[1] = (char)('0' + (mm % 10));
  out[2] = ':';
  out[3] = (char)('0' + (ss / 10));
  out[4] = (char)('0' + (ss % 10));
  out[5] = '\0';
}

// Map logical cell (A,F) -> actual motor amplitude/frequency percentages.
static void controller_command_cell(int aIndex, int fIndex)
{
  if (aIndex < 0)
    aIndex = 0;
  if (aIndex > 4)
    aIndex = 4;
  if (fIndex < 0)
    fIndex = 0;
  if (fIndex > 4)
    fIndex = 4;

  // static const uint8_t amp_levels[5] = {5, 20, 40, 60, 80};
  // static const uint8_t freq_levels[5] = {5, 20, 40, 60, 80};

  // uint8_t amp = amp_levels[aIndex];
  // uint8_t freq = freq_levels[fIndex];

  curA = aIndex;
  curF = fIndex;

  command_motor(curA, curF);
  // ---- CALM detection (A1F1 == indices 0,0) ----
  // We only count calm if we are NOT in panic mode (panic currently forces A1F1).
  if (!g_calm_reached && !panic_mode && g_algo_start_ms > 0.0 && curA == 0 && curF == 0)
  {
    g_calm_reached = 1;
    g_calm_elapsed_ms = (int)(now_msec() - g_algo_start_ms);
    log_printf("[A] CALM reached in %d ms\n", g_calm_elapsed_ms);
  }
}

// improvement tests (from sim)
static int heartbeat_improved(int bpm_now)
{
  if (ctrl_lastBPM <= 0)
    return 0;
  if (ctrl_lastBPM - bpm_now >= thresholdBPM)
    return 1;
  return 0;
}

static int crying_improved(int cry_now)
{
  if (cry_now <= thresholdCRY)
    return 1;
  if (ctrl_lastCRY > 0 && (ctrl_lastCRY - cry_now >= thresholdCRY))
    return 1;
  return 0;
}

// register anchor cell
static void register_anchor(int a, int f)
{
  if (a < 0 || a > 4 || f < 0 || f > 4)
    return;

  if (anchorMatrix[a][f] == 0)
  {
    anchorLevel++;
    anchorMatrix[a][f] = 10 - anchorLevel;
    log_printf("[A] set A%d F%d as anchor L%d\n",
               a + 1, f + 1, anchorLevel);
  }
}

// One controller step for
// This function is called every control cycle with the latest BPM and CRY and decides what to command on the motor grid.
// Yes this is extensively documented so that everyone can understand. Yes including me.
// bpm_now is the current heartbeat in BPM, cry_now is the current crying level (%) both are measured by the submodules, hopefully.
static void controller_step(int bpm_now, int cry_now)
{
  hit_wall = 0; // Detector flag for (AxF1 or A1Fx so we can be smart and reduce the delay to just the convergence time)

  // PANIC DETECTION USING VITALS
  // In this part we look only at BPM and CRY and decide whether the baby is in a panic state and we must enter panic_mode.
  // This matters because if we are in panic mode we need to go to K9 to start again. Currently the motors stop for testing purposes

  int big_jump = 0; // This variable will be set to 1 if the BPM suddenly jumps up a lot compared to the previous BPM

  if (ctrl_lastBPM > 0)                        // We only check for a BPM jump if we have a valid previous BPM
    big_jump = (bpm_now - ctrl_lastBPM >= 30); // Here we compute the difference between current BPM and last BPM, and set big_jump to 1 if the increase is 30 BPM or more.

  if (!panic_mode) // We only re-check panic conditions if we are not already in panic mode; once in panic, we stay there until its reseted somehow (not implement rk).
  {
    if (big_jump) // If any of our panic flags are true, panic.
    {
      panic_mode = 1; // We now enter panic mode, meaning that the rest of this function will follow the panic-mode path instead of the normal algorithm.

      log_printf("[A] PANIC(BPM=%d, CRY=%d)\n", bpm_now, cry_now); // We log a message so we can see exactly when and with what values the panic was triggered.
    }
  }

  // PANIC MODE: FREEZE MOTORS (Currently)
  // When panic_mode is active, we stop exploring the (A, F) grid and keep the cradle in a fixed safe motor state.

  if (panic_mode)
  {
    controller_command_cell(0, 0); // We command the cell at indices (A=0, F=0)

    ctrl_lastBPM = bpm_now; // We still update ctrl_lastBPM to the current BPM so history and logs remain up to date even during panic.
    ctrl_lastCRY = cry_now; // We also update ctrl_lastCRY to the current crying level for the same reason.
    return;                 // We leave the function early because, in panic mode, we do not want to run the normal inverse-model algorithm anymore.
  }

  // NORMAL MODE: CHECK WHETHER THE LAST MOVE HELPED OR NOT
  // Since we are not in panic, we now look at whether the last motor command improved the baby’s state.

  int improved = 0; // This will be set to 1 if the helper functions say that the situation actually got better after the last move.
  int same = 0;     // This will be set to 1 if the situation is considered stable

  if (bpm_now < 150 && (cry_now < 52 && cry_now > 15)) // If the current BPM is below 150, we stop using heart rate as its delayed and focus more on crying as an indicator of stress.
  {
    is_crying_activated = 1;             // We record that in this regime we are using crying as the primary signal to measure improvement.
    improved = crying_improved(cry_now); // We call crying_improved with the current CRY value. returns 1 if crying suggests improvement.
  }
  else // If BPM is 150 or higher, the heart rate is used since crying is always %100 here
  {
    is_crying_activated = 0;                // We record that, in this regime, we are using BPM as the primary indicator of improvement.
    improved = heartbeat_improved(bpm_now); // We call heartbeat_improved with the current BPM value. returns 1 if BPM suggests improvement
  }

  if (ctrl_lastBPM > 0) // We  attempt a “stability” check (if we have a valid previous BPM value otherwise we cannot compare)
  {
    int bpm_delta = abs(bpm_now - ctrl_lastBPM);                           // We calculate the absolute value of the difference between current BPM and last BPM to see how much it changed.
    int cry_delta = (ctrl_lastCRY >= 0) ? abs(cry_now - ctrl_lastCRY) : 0; // For CRY, we do a similar absolute difference if we have a valid previous value; otherwise we treat it as zero change.

    if (!is_crying_activated) // If we are currently in BPM-driven mode (using BPM to decide improvement),
    {
      if (bpm_delta <= 3) // then we consider the state “stable” if BPM changed by at most 3 beats since the last step.
      {
        log_printf("[A] HB stable Del(BPM)=%d\n", bpm_delta);
        if (lastMoveDir == 1) // If the last move we made on the grid was a LEFT move (direction 1),
          same = 1;           // we set same to 1, meaning we have a “stable after LEFT” pattern that we will react to with a special move i call reverse diagonal later.
      }
    }
    else // If we are in crying-driven mode (using CRY to decide improvement),
    {
      if (cry_delta == 0) // we treat the situation as stable only if crying did not change at all (difference equals zero).
      {
        log_printf("[A] CRY stable ΔCRY=%d\n", cry_delta); // We log that the crying level is stable and show the CRY difference (which is zero here).
        if (lastMoveDir == 1)                              // Again, this only matters if the last move direction was LEFT,
          same = 1;                                        // so we set same to 1 in that case to remember the “stable after LEFT” condition.
      }
    }
  }

  // ANCHOR SYNC WHEN IDLE (lastMoveDir == 0)
  // Anchors are positions on the grid that we know are in the solution path
  // when idle we make sure our stored anchor matches our current position.

  if (lastMoveDir == 0) // If lastMoveDir is 0, it means we are not in the middle of a move and are sitting on some anchor position.
  {
    if (anchorA_mem != curA || anchorF_mem != curF) // If the anchor stored in memory does not match our current (curA, curF) on the grid,
    {
      anchorA_mem = curA;      // we update the stored anchor amplitude index to the current A index.
      anchorF_mem = curF;      // we also update the stored anchor frequency index to the current F index.
      triedLeftFromAnchor = 0; // We reset the flag indicating whether we have tried going LEFT from this anchor, so it becomes allowed again.
      triedUpFromAnchor = 0;   // We also reset the flag indicating whether we have tried going UP from this anchor.

      register_anchor(anchorA_mem, anchorF_mem); // We call register_anchor to tell the rest of the system that (curA, curF) is now our chosen anchor cell.
      // this will later be used to follow a predetermined path to solution if a panic jump is caused to save time
    }
  }

  // FIRST MOVE FROM AN ANCHOR (when lastMoveDir == 0)
  // From an anchor, the algorithm chooses which neighbour to explore first (LEFT or UP).

  if (lastMoveDir == 0) // We are in the idle state, so now we decide the first exploration step from this anchor.
  {
    prevA = curA; // We store the current amplitude index as prevA, so we can return here later if needed.
    prevF = curF; // We also store the current frequency index as prevF for the same reason.
    if (!triedLeftFromAnchor && curF == 0)
    {
      hit_wall = 1; // wanted to try LEFT but wall
      controller_command_cell(curA - 1, curF);
      log_printf("[A] Hit left wall\n");
    }
    else if (!triedUpFromAnchor && curA == 0)
    {
      hit_wall = 1; // wanted to try UP but wall
      controller_command_cell(curA, curF - 1);
      log_printf("[A] Hit upper wall\n");
    }
    // just to be sure we still check vitals after we hit a wall instead of just going down.

    else if (!triedLeftFromAnchor && curF > 0) // If we have not already tried going LEFT from this anchor and we are not at the left border of the grid (F > 0),
    {
      lastMoveDir = 1;         // We set lastMoveDir to 1 to remember that we are now making a LEFT move.
      triedLeftFromAnchor = 1; // We also mark that from this anchor, LEFT has now been attempted, so we do not retry it immediately later.

      log_printf("[A] TRY-> LEFT from A%d F%d\n", curA + 1, curF + 1);

      controller_command_cell(curA, curF - 1); // We send the actual motor command to move to the cell with the same A index and F index decreased by one (one step LEFT on the grid).

      ctrl_lastBPM = bpm_now; // After issuing the command, we record the current BPM so that next time we can compare and see if there was improvement.
      ctrl_lastCRY = cry_now; // We also record the current CRY for the same comparison on the next step.
      return;                 // We return immediately, because we want to wait and see how this LEFT move changes the baby’s vitals before doing anything else.
    }
    else if (!triedUpFromAnchor && curA > 0) // If LEFT is not available or already tried, but we have not tried UP and we are not at the top row (A > 0),
    {
      lastMoveDir = 2;       // We set lastMoveDir to 2 to indicate that our next move is an UP move.
      triedUpFromAnchor = 1; // We mark that from this anchor, UP has been attempted, to avoid repeating it unnecessarily.

      log_printf("[A] Blocked-> UP from A%d F%d\n", curA + 1, curF + 1); // We log that our TRY move from this anchor is UP, and note that LEFT was already tried or blocked.

      controller_command_cell(curA - 1, curF); // We send the motor command to move to the neighbour above, which has A index decreased by one and the same F index.

      ctrl_lastBPM = bpm_now; // We store the BPM we saw before this UP move so that we can check later if it improved things.
      ctrl_lastCRY = cry_now; // We also store the CRY level for the same reason.
      return;                 // We return here, again to wait for the effect of this UP move on the vitals.
    }
    else if (curA + 1 == 1 && curF + 1 == 1) // If neither LEFT nor UP is available (or both have already been tried from this anchor),
    {
      log_printf("[A] BABY CALM holding A%d F%d\n", curA + 1, curF + 1);

      ctrl_lastBPM = bpm_now; // Even though we are not moving, we still update the last BPM value to what we just measured.
      ctrl_lastCRY = cry_now; // And we also update the last CRY value.
      return;                 // We exit the function while staying at this anchor, just monitoring the baby’s state.
    }
    else // If neither LEFT nor UP is available (or both have already been tried from this anchor),
    {
      log_printf("[A] Fatal Error! holding A%d F%d\n", curA + 1, curF + 1);

      ctrl_lastBPM = bpm_now; // Even though we are not moving, we still update the last BPM value to what we just measured.
      ctrl_lastCRY = cry_now; // And we also update the last CRY value.
      return;                 // We exit the function while staying at this anchor, just monitoring the baby’s state.
    }
  }

  // WE HAVE A LAST MOVE (lastMoveDir != 0) // If we reach here, it means we are returning after having commanded a move in the previous step.

  if (improved) // If the helper functions said that the last move improved the situation,
  {
    int anchorA = curA; // We now treat the current A index (where we ended up) as a new anchor amplitude index.
    int anchorF = curF; // We also treat the current F index as a new anchor frequency index.

    log_printf("[A] IMPROVED -> anchor A%d F%d\n", anchorA + 1, anchorF + 1);

    register_anchor(anchorA, anchorF); // We tell the anchor-management logic that this cell (anchorA, anchorF) should be added or updated as an anchor on the path.

    if (anchorA_mem != anchorA || anchorF_mem != anchorF) // If our remembered anchor position does not yet match this new anchor,
    {
      anchorA_mem = anchorA;   // we store the new anchor amplitude index in anchorA_mem.
      anchorF_mem = anchorF;   // and the new anchor frequency index in anchorF_mem.
      triedLeftFromAnchor = 0; // We reset the “tried left” flag, because this is a fresh anchor and we can try LEFT from it again.
      triedUpFromAnchor = 0;   // We also reset the “tried up” flag for the same reason.
    }

    prevA = anchorA; // We also store this anchor as prevA so that, if future moves fail, we can backtrack to it.
    prevF = anchorF; // And we store it as prevF for backtracking in frequency.

    if (anchorF > 0) // If we are not at the left border, we can try going further LEFT from this new anchor.
    {
      lastMoveDir = 1;         // We set the last move direction to LEFT again, as we are planning a follow-up LEFT move.
      triedLeftFromAnchor = 1; // We mark that LEFT has been tried from this anchor so we do not keep repeating it forever.

      log_printf("[A] IMPROVED-> LEFT from A%d F%d\n", anchorA + 1, anchorF + 1); // We log that, because the last move was good, we are going to continue exploring by moving LEFT from this new anchor.

      controller_command_cell(anchorA, anchorF - 1); // We command the motor module to move to the cell one step LEFT of the current anchor position.
      // we can shorten delays if borders are hit since there is only going to remain one path to solution so we wouldnt need to wait for the whole heartbeat delay and just the convergence delay. I just dont think this will happen.
    }
    else if (anchorA > 0) // Otherwise, if LEFT is impossible but we can still move UP (not at top boundary),
    {
      lastMoveDir = 2; // We set the next move direction to UP.
      // Note: we do not mark triedUpFromAnchor here, but we could if we want symmetric behaviour.

      log_printf("[A] IMPROVED-> try UP from A%d F%d\n", anchorA + 1, anchorF + 1); // We log that we improved and now we will try moving UP from this anchor instead.

      controller_command_cell(anchorA - 1, anchorF); // We command a move to the cell directly above this anchor (one step lower in A index).
    }

    ctrl_lastBPM = bpm_now; // After planning the next move, we store the current BPM so we can judge the effect in the next step.
    ctrl_lastCRY = cry_now; // And we also store the current crying level for the same purpose.
    return;                 // We exit here since the next decision will be made after we see new vitals.
  }
  else // If improved is 0, it means the last move did not make things better (it might be the same or worse).
  {
    // HANDLE NO-IMPROVEMENT (SAME OR WORSE) // We now decide whether to try a special reverse-diagonal move or just backtrack.

    if (same && lastMoveDir == 1) // If the state is considered “stable” and the last move direction was LEFT (dir=1),
    {
      int anchorA = prevA; // we use prevA as the anchor A index from which we came before that LEFT move.
      int anchorF = prevF; // and prevF as the anchor F index from before that LEFT move.

      if (anchorA > 0) // If we can still move UP from that previous anchor (i.e., we are not at the top row),
      {
        log_printf("[A] SAME-> R.D from A%d F%d\n", anchorA + 1, anchorF + 1); // We log that we detected the “same after left” pattern and will now try a reverse diagonal step from that anchor.

        lastMoveDir = 2;       // We set lastMoveDir to 2 because the reverse diagonal involves an UP move from the previous anchor.
        triedUpFromAnchor = 1; // We mark that, from this anchor, we are now trying UP so we do not keep repeating it unnecessarily.

        prevA = curA; // We store the current A index as prevA so that if this reverse diagonal is bad, we can backtrack back here.
        prevF = curF; // We also store the current F index as prevF for symmetrical backtracking.

        controller_command_cell(anchorA - 1, anchorF); // We execute the reverse diagonal by commanding the cell that is one step UP from the previous anchor.

        ctrl_lastBPM = bpm_now; // We update ctrl_lastBPM to remember the BPM at the moment we made this reverse diagonal decision.
        ctrl_lastCRY = cry_now; // And we also update ctrl_lastCRY to remember the CRY level at this moment.
        return;                 // We return so that on the next call we can see if this reverse diagonal move improved things.
      }
    }

    int anchorA = prevA; // If the special case above does not apply or is impossible, we prepare to backtrack to the previous anchor’s A index.
    int anchorF = prevF; // And we prepare to backtrack to the previous anchor’s F index.

    if (anchorA != curA || anchorF != curF) // If we are not already at that previous anchor cell,
    {
      log_printf("[A] NO IMPROVEMENT -> A%d F%d\n", anchorA + 1, anchorF + 1); // We log that there was no improvement and that we are backtracking to that anchor, including the direction we came from.

      controller_command_cell(anchorA, anchorF); // We send the command to move the motor state back exactly to the previous anchor cell on the grid.
    }

    curA = anchorA; // We update our current amplitude index to the anchor amplitude index we backtracked to.
    curF = anchorF; // We update our current frequency index to the anchor frequency index we backtracked to.

    lastMoveDir = 0; // We reset lastMoveDir to 0, indicating that we are now idle at an anchor and ready for the next “first move” decision.

    ctrl_lastBPM = bpm_now; // We store the current BPM as the last BPM for the next control step comparison.
    ctrl_lastCRY = cry_now; // We store the current crying level as the last CRY for the next comparison as well.
    return;                 // We exit the function; the next call will start again from an anchor in idle state.
  }
}

// Ctrl+C handler
static void handle_sigint(int sig __attribute__((unused)))
{
  // Stop HUD from drawing anything during shutdown
  g_log_enabled = 0;

  // Print to normal stdout only (no display drawing)
  printf("\n Exited\n");

  // Optional: you may skip the FillScreen; it is not needed on Ctrl+C.
  displayFillScreen(&g_disp, RGB_BLACK);

  display_destroy(&g_disp);
  switches_destroy();
  buttons_destroy();
  pynq_destroy();
  exit(0);
}

static void restart_program(void)
{
  // Prevent Ctrl+C during restart teardown/exec
  signal(SIGINT, SIG_IGN);

  // Optional: clear screen as a UX cue (safe to skip if you suspect display code)
  displayFillScreen(&g_disp, RGB_BLACK);

  // Execute self (argv[0] must be non-NULL)
  execl("/proc/self/exe", "/proc/self/exe", (char *)NULL);

  // If execl returns, it failed
  perror("execl failed");
  _exit(127);
}

int main(void)
{
  // init Ctrl+C clean-up
  signal(SIGINT, handle_sigint);

  // PYNQ + UART + IO init
  pynq_init();
  uart_init(UART_CH);
  uart_reset_fifos(UART_CH);
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);
  switches_init();
  buttons_init();

  // display + font
  display_init(&g_disp);
  display_set_flip(&g_disp, true, true);
  displayFillScreen(&g_disp, RGB_BLACK);

  uint8_t glyph[FontxGlyphBufSize];
  InitFontx(g_fx, "/boot/ILGH16XB.FNT", "");
  GetFontx(g_fx, 0, glyph, &g_fw, &g_fh);
  displaySetFontDirection(&g_disp, TEXT_DIRECTION0);

  int x = 6;
  int y = g_fh * 1;

  // MODE 1: communication demo (random) (switch 0)
  // if (get_switch_state(0) == 1)
  // {
  //   draw_text(&g_disp, g_fx, x, y, "COMMUNICATION DEMO MODE[R]", RGB_GREEN);
  //   y += g_fh;
  //   draw_text(&g_disp, g_fx, x, y, "[BOOT]: requesting random", RGB_WHITE);
  //   y += g_fh;
  //   draw_text(&g_disp, g_fx, x, y, "data from all modules", RGB_WHITE);
  //   y += g_fh;
  //   y += g_fh;

  //   int v;
  //   int successCounter = 0;

  //   // heartbeat
  //   draw_text(&g_disp, g_fx, x, y, "Heartbeat @1: ...", RGB_WHITE);
  //   v = request_random(HRTBT);
  //   clear_text_line(&g_disp, y, g_fh, RGB_BLACK);
  //   if (v >= 0)
  //   {
  //     successCounter++;
  //     char buf[32], num[16];
  //     strcpy(buf, "HB @1: ALIVE:");
  //     itoa_u((unsigned)v, num);
  //     strcat(buf, num);
  //     draw_text(&g_disp, g_fx, x, y, buf, RGB_GREEN);
  //   }
  //   else
  //   {
  //     draw_text(&g_disp, g_fx, x, y, "HB @1: FAILED", RGB_RED);
  //   }
  //   y += g_fh;

  //   // crying
  //   draw_text(&g_disp, g_fx, x, y, "Crying @2: ...", RGB_WHITE);
  //   v = request_random(CRY);
  //   clear_text_line(&g_disp, y, g_fh, RGB_BLACK);
  //   if (v >= 0)
  //   {
  //     successCounter++;
  //     char buf1[32], num1[16];
  //     strcpy(buf1, "CRY @2: ALIVE:");
  //     itoa_u((unsigned)v, num1);
  //     strcat(buf1, num1);
  //     draw_text(&g_disp, g_fx, x, y, buf1, RGB_GREEN);
  //   }
  //   else
  //   {
  //     draw_text(&g_disp, g_fx, x, y, "CRY @2: FAILED", RGB_RED);
  //   }
  //   y += g_fh;

  //   // motor
  //   draw_text(&g_disp, g_fx, x, y, "Motor @3: ...", RGB_WHITE);
  //   motor_send_random();
  //   clear_text_line(&g_disp, y, g_fh, RGB_BLACK);

  //   successCounter++;
  //   char buf[48], a[16], f[16];
  //   strcpy(buf, "MTR @3: SENT A:");
  //   itoa_u((unsigned)g_amp, a);
  //   strcat(buf, a);
  //   strcat(buf, " F:");
  //   itoa_u((unsigned)g_freq, f);
  //   strcat(buf, f);
  //   draw_text(&g_disp, g_fx, x, y, buf, RGB_YELLOW);
  //   y += g_fh;
  //   y += g_fh;

  //   if (successCounter == 1)
  //   {
  //     draw_text(&g_disp, g_fx, x, y, "DEMO FAILED", RGB_RED);
  //     y += g_fh;
  //   }
  //   if (successCounter == 3)
  //   {
  //     draw_text(&g_disp, g_fx, x, y, "DEMO PASSED", RGB_GREEN);
  //     y += g_fh;
  //   }

  //   // stay here until switch 0 is turned off
  //   while (get_switch_state(0) == 1)
  //     sleep_msec(10);

  //   display_destroy(&g_disp);
  //   switches_destroy();
  //   buttons_destroy();
  //   pynq_destroy();
  //   return EXIT_SUCCESS;
  // }

  // MODE 1: MANUAL VITALS DEMO (switch 0)
  // Buttons:
  //  B0: BPM-   (minus 10)
  //  B1: BPM+   (plus 10)
  //  B2: CRY-   (minus 10)
  //  B3: CRY+   (plus 10)
  if (get_switch_state(0) == 1)
  {
    // Start values chosen to show algorithm behavior immediately
    uint8_t demo_bpm = 220; // likely HB-driven
    uint8_t demo_cry = 100; // typical "stressed" cry
    // int prev_b0 = 0, prev_b1 = 0, prev_b2 = 0, prev_b3 = 0;

    displayFillScreen(&g_disp, RGB_BLACK);
    y = g_fh * 1;

    draw_text(&g_disp, g_fx, x, y, "DEMO MODE: MANUAL VITALS", RGB_GREEN);
    y += g_fh;
    draw_text(&g_disp, g_fx, x, y, "3CRY+ 2CRY- 1BPM+ 0BPM-", RGB_WHITE);
    y += g_fh;

    // Reserve fixed HUD lines (so we can clear/redraw in place)
    int y_demo_bpm = y;
    y += g_fh;
    int y_demo_cry = y;
    y += g_fh;
    int y_demo_mode = y;
    y += g_fh;
    int y_demo_cell = y;
    y += g_fh;
    int y_demo_mtr = y;
    y += g_fh;
    int y_demo_panic = y;
    y += g_fh;
    int y_demo_time = y;
    y += g_fh;

    // Put log region below HUD (optional; uses your existing hud_log)
    g_log_x = x;
    g_log_y_start = y + g_fh;

    // *** NEW *** define end of cyan log region (do not touch HUD above)
    g_log_y_end = DISPLAY_HEIGHT; // default: use bottom of screen
    // If you want to reserve a safety margin of 2 px:
    g_log_y_end = DISPLAY_HEIGHT - 2;

    if (g_log_y_start > g_log_y_end - g_fh)
      g_log_y_start = g_log_y_end - g_fh;

    g_log_y = g_log_y_start;
    g_log_enabled = 1;

    // Ensure controller starts from known state (A5 F5)
    curA = 4;
    curF = 4;
    prevA = curA;
    prevF = curF;
    lastMoveDir = 0;
    is_crying_activated = 0;
    ctrl_lastBPM = -1;
    ctrl_lastCRY = -1;
    anchorA_mem = -1;
    anchorF_mem = -1;
    triedLeftFromAnchor = 0;
    triedUpFromAnchor = 0;
    memset(anchorMatrix, 0, sizeof(anchorMatrix)); // idek wtf this is but hopefully we arent using the anchor matrix
    anchorLevel = 0;
    panic_mode = 0;

    // Put motor to start cell so the output line is meaningful immediately
    controller_command_cell(curA, curF);

    g_algo_start_ms = now_msec();
    g_calm_reached = 0;
    g_calm_elapsed_ms = 0;

    // Run demo until switch 0 is turned off
    int cry_flag = 0;
    while (get_switch_state(0) == 1)
    {
      // --- Buttons (not-edge-detected) ---
      int b0 = get_button_state(0);
      int b1 = get_button_state(1);
      int b2 = get_button_state(2);
      int b3 = get_button_state(3);

      // Level-based: if you hold the button until the next loop iteration, it will apply.
      if (b0)
        demo_bpm = (uint8_t)clampi((int)demo_bpm - 10, 60, 240);
      if (b1)
        demo_bpm = (uint8_t)clampi((int)demo_bpm + 10, 60, 240);
      if (b2)
        demo_cry = (uint8_t)clampi((int)demo_cry - 10, 0, 100);
      if (b3)
        demo_cry = (uint8_t)clampi((int)demo_cry + 10, 0, 100);

      if (demo_bpm < 150 && !cry_flag)
      {
        demo_cry = 52;
        cry_flag = 1;
      }

      // --- Run real decision logic with injected vitals ---
      controller_step((int)demo_bpm, (int)demo_cry);

      // --- Draw HUD lines (clear then redraw fixed positions) ---
      clear_text_line(&g_disp, y_demo_bpm, g_fh, RGB_BLACK);
      clear_text_line(&g_disp, y_demo_cry, g_fh, RGB_BLACK);
      clear_text_line(&g_disp, y_demo_mode, g_fh, RGB_BLACK);
      clear_text_line(&g_disp, y_demo_cell, g_fh, RGB_BLACK);
      clear_text_line(&g_disp, y_demo_mtr, g_fh, RGB_BLACK);
      clear_text_line(&g_disp, y_demo_panic, g_fh, RGB_BLACK);
      clear_text_line(&g_disp, y_demo_time, g_fh, RGB_BLACK);

      char buf[96], num[16];

      // BPM line
      strcpy(buf, "[DEMO] BPM=");
      itoa_u(demo_bpm, num);
      strcat(buf, num);
      draw_text(&g_disp, g_fx, x, y_demo_bpm, buf, RGB_WHITE);

      // CRY line
      strcpy(buf, "[DEMO] CRY=");
      itoa_u(demo_cry, num);
      strcat(buf, num);
      strcat(buf, "%");
      draw_text(&g_disp, g_fx, x, y_demo_cry, buf, RGB_WHITE);

      // Regime line (uses your global is_crying_activated)
      if (is_crying_activated)
        strcpy(buf, "[MODE] CRY driven"); // shorter delay
      else
        strcpy(buf, "[MODE] HB driven");
      draw_text(&g_disp, g_fx, x, y_demo_mode, buf, RGB_YELLOW);

      // Controller output cell (curA/curF are your controller state)
      strcpy(buf, "[CTRL] Decided Cell: A");
      itoa_u((unsigned)(curA + 1), num);
      strcat(buf, num);
      strcat(buf, " F");
      itoa_u((unsigned)(curF + 1), num);
      strcat(buf, num);
      draw_text(&g_disp, g_fx, x, y_demo_cell, buf, RGB_CYAN);

      // Motor command output (g_amp/g_freq are your command outputs)
      strcpy(buf, "[MOTOR] CMD-> A:");
      itoa_u((unsigned)g_amp, num);
      strcat(buf, num);
      strcat(buf, "% F:");
      itoa_u((unsigned)g_freq, num);
      strcat(buf, num);
      strcat(buf, "%");
      draw_text(&g_disp, g_fx, x, y_demo_mtr, buf, RGB_WHITE);

      // Panic indicator
      strcpy(buf, "[PANIC] ");
      strcat(buf, panic_mode ? "TRIGGERED" : "NOT TRIGGERED");
      draw_text(&g_disp, g_fx, x, y_demo_panic, buf, panic_mode ? RGB_RED : RGB_GREEN);
      int elapsed_ms = g_calm_reached ? g_calm_elapsed_ms : (int)(now_msec() - g_algo_start_ms);
      char tbuf[8];
      fmt_mmss(elapsed_ms, tbuf);

      strcpy(buf, "[TIME] ");
      strcat(buf, tbuf);
      strcat(buf, g_calm_reached ? " (CALM)" : "");
      draw_text(&g_disp, g_fx, x, y_demo_time, buf, g_calm_reached ? RGB_GREEN : RGB_WHITE);

      int delay_ms_s;
      if (hit_wall)
      {
        delay_ms_s = CONVERGENCE_DELAY;
      }
      else if (is_crying_activated)
      {
        delay_ms_s = CRYING_DELAY;
      }
      else
      {
        delay_ms_s = HEARTBEAT_DELAY;
      }

      sleep_msec(delay_ms_s);
    }

    // Exit demo mode cleanly
    g_log_enabled = 0;
    display_destroy(&g_disp);
    switches_destroy();
    buttons_destroy();
    pynq_destroy();
    return EXIT_SUCCESS;
  }

  // MODE 2: live communication demo (switch 1)
if (get_switch_state(1) == 1)
{
  displayFillScreen(&g_disp, RGB_BLACK);
  y = g_fh * 1;

  draw_text(&g_disp, g_fx, x, y, "COMMUNICATION DEMO MODE", RGB_GREEN);
  y += g_fh;

  // --- PING STATUS LINES (no new functions; reuse boot_ping) ---
  int y_p1 = y; y += g_fh;
  int y_p2 = y; y += g_fh;
  int y_p3 = y; y += g_fh;

  draw_text(&g_disp, g_fx, x, y_p1, "HB @1: pinging...", RGB_WHITE);
  int hb_ok = boot_ping(HRTBT);
  clear_text_line(&g_disp, y_p1, g_fh, RGB_BLACK);
  draw_text(&g_disp, g_fx, x, y_p1, hb_ok ? "HB @1: ALIVE" : "HB @1: MISSING",
            hb_ok ? RGB_GREEN : RGB_RED);

  draw_text(&g_disp, g_fx, x, y_p2, "CRY @2: pinging...", RGB_WHITE);
  int cry_ok = boot_ping(CRY);
  clear_text_line(&g_disp, y_p2, g_fh, RGB_BLACK);
  draw_text(&g_disp, g_fx, x, y_p2, cry_ok ? "CRY @2: ALIVE" : "CRY @2: MISSING",
            cry_ok ? RGB_GREEN : RGB_RED);

  draw_text(&g_disp, g_fx, x, y_p3, "MTR @3: pinging...", RGB_WHITE);
  int mtr_ok = boot_ping(MTR);
  clear_text_line(&g_disp, y_p3, g_fh, RGB_BLACK);
  draw_text(&g_disp, g_fx, x, y_p3, mtr_ok ? "MTR @3: ALIVE" : "MTR @3: MISSING",
            mtr_ok ? RGB_GREEN : RGB_RED);

  // --- HUD lines for live values ---
  y += g_fh; // spacer
  int y_live_hb1  = y; y += g_fh;
  int y_live_cry1 = y; y += g_fh;
  int y_live_mtr1 = y; y += g_fh;

  uint8_t amp = 0;
  uint8_t freq = 0;
  int prev_b0 = 0, prev_b1 = 0, prev_b3 = 0;

  while (get_switch_state(1) == 1)
  {
    // Only request if that module responded to ping (keeps demo clean)
    if (hb_ok)
    {
      int vhb = request_heartbeat();
      if (vhb >= 0) last_bpm = (uint8_t)vhb;
    }

    if (cry_ok)
    {
      int vcr = request_crying();
      if (vcr >= 0) last_cry = (uint8_t)vcr;
    }

    int b0 = get_button_state(0);
    int b1 = get_button_state(1);
    int b3 = get_button_state(3);

    if (b3 && !prev_b3)
      restart_program();

    // Only command motor if it pinged OK
    if (mtr_ok)
    {
      if (b0 && !prev_b0)
      {
        amp = 100;
        freq = 70;
        command_motor(amp, freq);
      }
      else if (b1 && !prev_b1)
      {
        amp = 80;
        freq = 60;
        command_motor(amp, freq);
      }
    }

    prev_b0 = b0;
    prev_b1 = b1;
    prev_b3 = b3;

    clear_text_line(&g_disp, y_live_hb1, g_fh, RGB_BLACK);
    clear_text_line(&g_disp, y_live_cry1, g_fh, RGB_BLACK);
    clear_text_line(&g_disp, y_live_mtr1, g_fh, RGB_BLACK);

    char buf[64], num[16];

    strcpy(buf, "[HB] bpm=");
    itoa_u(last_bpm, num);
    strcat(buf, num);
    draw_text(&g_disp, g_fx, x, y_live_hb1, buf, RGB_WHITE);

    strcpy(buf, "[C] cry=");
    itoa_u(last_cry, num);
    strcat(buf, num);
    strcat(buf, "%");
    draw_text(&g_disp, g_fx, x, y_live_cry1, buf, RGB_WHITE);

    strcpy(buf, "[MOTOR] sent= A:");
    itoa_u(amp, num);
    strcat(buf, num);
    strcat(buf, "%  F:");
    itoa_u(freq, num);
    strcat(buf, num);
    strcat(buf, "%");
    draw_text(&g_disp, g_fx, x, y_live_mtr1, buf, RGB_WHITE);

    sleep_msec(20);
  }

  // IMPORTANT: do NOT destroy and return here.
  // Switching off SW1 should fall through to MODE 3.
  displayFillScreen(&g_disp, RGB_BLACK);
}


  // MODE 3: REAL DECISION-MAKING MODULE (default)

  displayFillScreen(&g_disp, RGB_BLACK);
  y = g_fh * 1;
  draw_text(&g_disp, g_fx, x, y, "DECISION MAKING MODULE", RGB_GREEN);
  y += g_fh;
  draw_text(&g_disp, g_fx, x, y, "[BOOT]: pinging modules...", RGB_WHITE);
  y += g_fh;

  int y_hb = y;
  y += g_fh;
  int y_cr = y;
  y += g_fh;
  int y_mt = y;
  y += g_fh;

  // ping modules
  draw_text(&g_disp, g_fx, x, y_hb, "HB @1: ...", RGB_WHITE);
  int hb_ok = boot_ping(HRTBT);
  clear_text_line(&g_disp, y_hb, g_fh, RGB_BLACK);
  draw_text(&g_disp, g_fx, x, y_hb, hb_ok ? "HB @1: ALIVE" : "HB @1: MISSING",
            hb_ok ? RGB_GREEN : RGB_RED);

  draw_text(&g_disp, g_fx, x, y_cr, "CRY @2: ...", RGB_WHITE);
  int cry_ok = boot_ping(CRY);
  clear_text_line(&g_disp, y_cr, g_fh, RGB_BLACK);
  draw_text(&g_disp, g_fx, x, y_cr, cry_ok ? "CRY @2: ALIVE" : "CRY @2: MISSING",
            cry_ok ? RGB_GREEN : RGB_RED);

  draw_text(&g_disp, g_fx, x, y_mt, "MTR @3: ...", RGB_WHITE);
  int mtr_ok = boot_ping(MTR);
  clear_text_line(&g_disp, y_mt, g_fh, RGB_BLACK);
  draw_text(&g_disp, g_fx, x, y_mt, mtr_ok ? "MTR @3: ALIVE" : "MTR @3: MISSING",
            mtr_ok ? RGB_GREEN : RGB_RED);

  // Reserve fixed HUD lines (clear/redraw in place)
  int y_live_hb = y;
  y += g_fh;
  int y_live_cry = y;
  y += g_fh;
  int y_live_mode = y;
  y += g_fh;
  int y_live_cell = y;
  y += g_fh;
  int y_live_mtr = y;
  y += g_fh;
  int y_live_panic = y;
  y += g_fh;
  int y_live_time = y;
  y += g_fh;

  // init controller start cell = A5 F5
  curA = 4;
  curF = 4;
  prevA = curA;
  prevF = curF;
  lastMoveDir = 0;
  g_algo_start_ms = now_msec();
  g_calm_reached = 0;
  g_calm_elapsed_ms = 0;

  // init on-screen log area *below* HUD, stay inside screen
  g_log_x = x;
  g_log_y_start = y + g_fh; // y is already advanced past the reserved HUD rows

  // controller log region ends at bottom of screen (leave everything else unchanged)
  g_log_y_end = DISPLAY_HEIGHT - 2;

  if (g_log_y_start > g_log_y_end - g_fh)
    g_log_y_start = g_log_y_end - g_fh;

  g_log_y = g_log_y_start;
  g_log_enabled = 1;

  //uint32_t last_poll_ms = 0;
  uint32_t last_step_ms = 0;

  // right after boot_ping() and before entering the while(1)
for (int i = 0; i < 50; i++) {           // ~1 second at 20ms
  int vhb = request_heartbeat();
  if (vhb >= 0) last_bpm = (uint8_t)vhb;

  int vcr = request_crying();
  if (vcr >= 0) last_cry = (uint8_t)vcr;

  if (last_bpm != 0 || last_cry != 0) break;
  sleep_msec(20);
}


  //int firstTime=1;
  // Main control loop
  while (1)
  {
    uint32_t now = (uint32_t)now_msec();

//     if(firstTime){
//   sleep_msec(HEARTBEAT_DELAY);
//   firstTime=0;
// }

    // restart
    if (get_button_state(3))
      restart_program();

    // (1) Poll vitals frequently
    int vhb = request_heartbeat();
      if (vhb >= 0)
        last_bpm = (uint8_t)vhb;
   
    last_cry=0;

    // (2) Run controller step on your intended cadence (4s or 10s)
    int step_period_ms;
    if (hit_wall)
      step_period_ms = CONVERGENCE_DELAY;
    else if (is_crying_activated)
      step_period_ms = CRYING_DELAY;
    else
      step_period_ms = HEARTBEAT_DELAY;

    if ((uint32_t)(now - last_step_ms) >= (uint32_t)step_period_ms)
    {
      last_step_ms = now;
      if (mtr_ok)
        controller_step((int)last_bpm, (int)last_cry);
    }

    // 3) HUD update and clear
    clear_text_line(&g_disp, y_live_hb, g_fh, RGB_BLACK);
    clear_text_line(&g_disp, y_live_cry, g_fh, RGB_BLACK);
    clear_text_line(&g_disp, y_live_mode, g_fh, RGB_BLACK);
    clear_text_line(&g_disp, y_live_cell, g_fh, RGB_BLACK);
    clear_text_line(&g_disp, y_live_mtr, g_fh, RGB_BLACK);
    clear_text_line(&g_disp, y_live_panic, g_fh, RGB_BLACK);
    clear_text_line(&g_disp, y_live_time, g_fh, RGB_BLACK);

    char buf[96], num[16];

    // HB
    strcpy(buf, "[HB] bpm=");
    itoa_u(last_bpm, num);
    strcat(buf, num);
    draw_text(&g_disp, g_fx, x, y_live_hb, buf, RGB_WHITE);

    // CRY
    strcpy(buf, "[C] cry=");
    itoa_u(last_cry, num);
    strcat(buf, num);
    strcat(buf, "%");
    draw_text(&g_disp, g_fx, x, y_live_cry, buf, RGB_WHITE);

    // MODE (uses is_crying_activated)
    if (is_crying_activated)
      strcpy(buf, "[MODE] CRY driven");
    else
      strcpy(buf, "[MODE] HB driven");
    draw_text(&g_disp, g_fx, x, y_live_mode, buf, RGB_YELLOW);

    // CELL (curA/curF)
    strcpy(buf, "[CTRL] Decided Cell: A");
    itoa_u((unsigned)(curA + 1), num);
    strcat(buf, num);
    strcat(buf, " F");
    itoa_u((unsigned)(curF + 1), num);
    strcat(buf, num);
    draw_text(&g_disp, g_fx, x, y_live_cell, buf, RGB_CYAN);

    // MOTOR (g_amp/g_freq)
    strcpy(buf, "[MOTOR] A:");
    itoa_u(g_amp, num);
    strcat(buf, num);
    strcat(buf, "% F:");
    itoa_u(g_freq, num);
    strcat(buf, num);
    strcat(buf, "%");
    draw_text(&g_disp, g_fx, x, y_live_mtr, buf, RGB_WHITE);

    // PANIC
    strcpy(buf, "[PANIC] ");
    strcat(buf, panic_mode ? "TRIGGERED" : "NOT TRIGGERED");
    draw_text(&g_disp, g_fx, x, y_live_panic, buf, panic_mode ? RGB_RED : RGB_GREEN);

    // TIME (and CALM marker)
    int elapsed_ms = g_calm_reached ? g_calm_elapsed_ms : (int)(now_msec() - g_algo_start_ms);
    char tbuf[8];
    fmt_mmss(elapsed_ms, tbuf);

    strcpy(buf, "[TIME] ");
    strcat(buf, tbuf);
    strcat(buf, g_calm_reached ? " (CALM)" : "");
    draw_text(&g_disp, g_fx, x, y_live_time, buf, g_calm_reached ? RGB_GREEN : RGB_WHITE);

    // Real-life reaction delay:
    // If crying-based regime: short delay (4 s)
    // If heartbeat-based regime: long delay (10 s) to respect TAU
    int delay_ms;
    if (hit_wall)
    {
      delay_ms = CONVERGENCE_DELAY;
    }
    else if (is_crying_activated)
    {
      delay_ms = CRYING_DELAY;
    }
    else
    {
      delay_ms = HEARTBEAT_DELAY;
    }

    sleep_msec(delay_ms);
    


  }

  // unreachable, but for completeness
  display_destroy(&g_disp);
  switches_destroy();
  buttons_destroy();
  pynq_destroy();
  return EXIT_SUCCESS;
}
