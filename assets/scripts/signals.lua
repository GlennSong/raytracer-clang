-- THE TRAFFIC SIGNALS' CYCLE (citysim; src/apps/citysim/traffic_signal.h, ADR-0060 / ADR-0109)
--
-- Every signalled junction cycles its phase groups -- each group's green, yellow, then all-red -- and closes the
-- cycle with a WALK window, every approach red while pedestrians cross. A group whose opposing approaches turn
-- across each other opens with a lead arrow (its green, then its yellow) before the shared green.
-- Seconds. With no file the citysim uses the same times built into C++ (SignalTiming -- a test keeps the two equal).
--
-- Keep a 2-group cycle near 34 s: longer cycles queue more cars per red than a small city block holds, and a
-- saturated ring of blocks is a deadlock no rule can undo.

signals = {
  green        = 10.0,
  yellow       = 2.0,
  all_red      = 1.5,
  walk         = 8.0,
  arrow        = 7.0,   -- a lead left arrow's green (the driver's right turn: traffic keeps left)
  arrow_yellow = 2.5,
}
