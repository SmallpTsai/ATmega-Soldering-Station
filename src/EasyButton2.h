#ifndef _EASYBUTTON_2_H_
#define _EASYBUTTON_2_H_

#include <EasyButton.h>

class EasyButton2 : public EasyButton
{
public:
  EasyButton2(uint8_t pin, uint32_t longpress_duration = 999999) : EasyButton(pin) {
    _lp_duration = longpress_duration;
    _last_last_change = 0;
  }

  bool read() {
    auto last_change = _last_change;
    auto result = EasyButton::read();
    if (_changed) {
        _last_last_change = last_change;
    }
    return result;
  }

  bool wasLongPressed() {
    return pressedFor(_lp_duration);
  }

  bool wasReleasedFromPress() {
    return wasReleased() && (_last_change < _last_last_change + _lp_duration);
  }

  bool wasReleasedFromLongPress() {
    return wasReleased() && (_last_change >= _last_last_change + _lp_duration);
  }

private:
  uint32_t _lp_duration;
  uint32_t _last_last_change;
};


#endif