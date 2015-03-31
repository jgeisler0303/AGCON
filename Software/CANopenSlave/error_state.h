#ifndef ERROR_STATE_H
#define ERROR_STATE_H

enum tx_error_state_enum {
  tx_no_error= 0,
  tx_warning,
  tx_all_busy,
  tx_passive,
  tx_error,
  tx_bus_off
};

#endif // ERROR_STATE_H
