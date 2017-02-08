import subprocess as sub
#                      brightness (int)    : min=0 max=255 step=1 default=128 value=117
#                        contrast (int)    : min=0 max=255 step=1 default=128 value=252
#                      saturation (int)    : min=0 max=255 step=1 default=128 value=128
#  white_balance_temperature_auto (bool)   : default=1 value=1
#                            gain (int)    : min=0 max=255 step=1 default=0 value=255
#            power_line_frequency (menu)   : min=0 max=2 default=2 value=2
# 				                                0: Disabled
# 			            	                    1: 50 Hz
#            				                    2: 60 Hz
#       white_balance_temperature (int)    : min=2000 max=6500 step=1 default=4000 value=4513 flags=inactive
#                       sharpness (int)    : min=0 max=255 step=1 default=128 value=128
#          backlight_compensation (int)    : min=0 max=1 step=1 default=0 value=0
#                   exposure_auto (menu)   : min=0 max=3 default=3 value=1
#        				                        1: Manual Mode
# 		        		                        3: Aperture Priority Mode
#               exposure_absolute (int)    : min=3 max=2047 step=1 default=250 value=80
#          exposure_auto_priority (bool)   : default=0 value=0
#                    pan_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
#                   tilt_absolute (int)    : min=-36000 max=36000 step=3600 default=0 value=0
#                  focus_absolute (int)    : min=0 max=250 step=5 default=0 value=0 flags=inactive
#                      focus_auto (bool)   : default=1 value=1
#                   zoom_absolute (int)    : min=100 max=500 step=1 default=100 value=100


class BaseProperty(object):
    dev_num = -1

    # HELPER CODE
    def _read_property(self, property_id):
        """
        Helper method: reads a given property from WebCam
        """
        cmd_str = "v4l2-ctl -d %d -C %s" % (self.dev_num, property_id)
        return self._parse_response(self._run_cmd(cmd_str), property_id)

    def _set_property(self, property_id, value):
        """
        Helper method: writes a given property to WebCam
        """
        cmd_str = "v4l2-ctl -d %d -c %s=%d" % (self.dev_num, property_id, value)
        return self._run_cmd(cmd_str)

    @staticmethod
    def _run_cmd(cmd_str):
        """
        Helper method: writes command to system
        """
        p = sub.Popen(cmd_str.split(), stdout=sub.PIPE, stderr=sub.PIPE)
        output, errors = p.communicate()
        return output, errors

    @staticmethod
    def _parse_response(response, property_id):
        """
        Helper method: tries to parse out the number
        """
        # TODO add error handling
        msg, err = response
        if property_id in msg:
            msg = msg.strip()
            msg = msg.split()
            msg = int(msg[1])
        return msg

    @staticmethod
    def _validate_range(min_val, max_val, val):
        """
        Helper method: validates that the given number is in the given range
        """
        # TODO Add check to check that value is integer or float
        if not min_val <= val <= max_val:
            error_msg = "Value %d is not in the valid range from %d->%d" % (val, min_val, max_val)
            raise ValueError(error_msg)


class AdjustableProperty(BaseProperty):
    name = ""
    _tag = ""
    min = None
    max = None
    default = None

    def __init__(self, name, tag, min_val, max_val, default):
        self.name = name
        self._tag = tag
        self.min = min_val
        self.max = max_val
        self.default = default

    def __repr__(self):
        return str(self.value)

    @property
    def value(self):
        return self._read_property(self._tag)

    @value.setter
    def value(self, value):
        self._validate_range(self.min, self.max, value)
        self._set_property(self._tag, value)

    def set_default(self):
        self.value = self.default

    def set_to_min(self):
        self.value = self.min

    def set_to_max(self):
        self.value = self.max


class AutoProperty(BaseProperty):
    _auto_name = None
    _auto_on_val = None
    _auto_off_val = None

    def __init__(self, auto_name, auto_on_val, auto_off_val):
        super(AutoProperty, self).__init__()
        self._auto_name = auto_name
        self._auto_on_val = auto_on_val
        self._auto_off_val = auto_off_val

    def __repr__(self):
        return str(self.auto)

    @property
    def auto(self):
        value = self._read_property(self._auto_name)
        return False if value is self._auto_off_val else True

    @auto.setter
    def auto(self, value):
        mode = self._auto_on_val if value else self._auto_off_val
        self._set_property(self._auto_name, mode)


class AutoAdjustableCamProperty(AdjustableProperty, AutoProperty):
    def __init__(self, name, tag, min_val, max_val, default, auto_name, auto_on_val, auto_off_val):
        AdjustableProperty.__init__(self, name, tag, min_val, max_val, default)
        AutoProperty.__init__(self, auto_name, auto_on_val, auto_off_val)


class C920WebCam(object):
    """
    Wrapper class to handle Logitech C920 WebCam adjustments.
    Note: Requires v2l2-util linux module
    """
    # Exposure values
    _auto_exposure_tag = "exposure_auto"
    _exposure_tag = "exposure_absolute"
    _exposure_manual_mode = 1
    _exposure_aperture_priority_mode = 3

    # White balance values
    _auto_white_balance_tag = "white_balance_temperature_auto"
    _white_balance_tag = "white_balance_temperature"

    # GAIN VALUES
    _gain_tag = "gain"

    # BRIGHTNESS VALUES
    _brightness_tag = "brightness"

    # CONTRAST VALUES
    _contrast_tag = "contrast"

    # SATURATION VALUES
    _saturation_tag = "saturation"

    # SHARPNESS VALUES
    _sharpness_tag = "sharpness"

    # ZOOM VALUES
    _zoom_tag = "zoom_absolute"

    # FOCUS VALUES
    _focus_tag = "focus_absolute"
    _auto_focus_tag = "focus_auto"

    # BACK LIGHT COMPENSATION
    _back_light_compensation_tag = "backlight_compensation"

    # POWER LINE FREQUENCY
    _power_line_freq_tag = "power_line_frequency"

    def __init__(self, dev_num):
        super(C920WebCam, self).__init__()
        self.dev_num = dev_num

        self._exposure = \
            AutoAdjustableCamProperty(name="EXPOSURE",
                                      tag=self._exposure_tag,
                                      min_val=3,
                                      max_val=2047,
                                      default=250,
                                      auto_name=self._auto_exposure_tag,
                                      auto_on_val=self._exposure_aperture_priority_mode,
                                      auto_off_val=self._exposure_manual_mode)

        self._white_balance = \
            AutoAdjustableCamProperty(name="WB",
                                      tag=self._white_balance_tag,
                                      min_val=2000,
                                      max_val=6500,
                                      default=4000,
                                      auto_name=self._auto_white_balance_tag,
                                      auto_on_val=1,
                                      auto_off_val=0)

        self._focus = \
            AutoAdjustableCamProperty(name="FOCUS",
                                      tag=self._focus_tag,
                                      min_val=0,
                                      max_val=255,
                                      default=0,
                                      auto_name=self._auto_focus_tag,
                                      auto_on_val=1,
                                      auto_off_val=0)

        self._gain = \
            AdjustableProperty(name="GAIN",
                               tag=self._gain_tag,
                               min_val=0,
                               max_val=255,
                               default=0)

        self._brightness = \
            AdjustableProperty(name="BRIGHTNESS",
                               tag=self._brightness_tag,
                               min_val=0,
                               max_val=255,
                               default=128)

        self._contrast = \
            AdjustableProperty(name="CONTRAST",
                               tag=self._contrast_tag,
                               min_val=0,
                               max_val=255,
                               default=128)

        self._saturation = \
            AdjustableProperty(name="SATURATION",
                               tag=self._saturation_tag,
                               min_val=0,
                               max_val=255,
                               default=128)

        self._sharpness = \
            AdjustableProperty(name="SHARPNESS",
                               tag=self._sharpness_tag,
                               min_val=0,
                               max_val=255,
                               default=128)

        self._zoom = \
            AdjustableProperty(name="ZOOM",
                               tag=self._zoom_tag,
                               min_val=0,
                               max_val=500,
                               default=100)

        self._power_line_freq = \
            AdjustableProperty(name="POWER LINE HZ",
                               tag=self._power_line_freq_tag,
                               min_val=0,
                               max_val=2,
                               default=0)

        self._back_light = \
            AutoProperty(auto_name=self._back_light_compensation_tag,
                         auto_on_val=1,
                         auto_off_val=0)

        self._set_device_num(dev_num)

    def _set_device_num(self, dev_num):
        for key, value in self.__dict__.iteritems():
            if isinstance(value, BaseProperty):
                value.dev_num = dev_num

    @property
    def powerline_frequency(self):
        """
        Returns the current power line frequency
        0: Disabled
        1: 50 Hz
        2: 60 Hz
        :return: current value
        :rtype: int
        """
        return self._power_line_freq

    @powerline_frequency.setter
    def powerline_frequency(self, value):
        """
        Sets the power frequency
        0: Disabled
        1: 50 Hz
        2: 60 Hz
        :param value: new power frequency
        :type value: int
        """
        self._power_line_freq.value = value

    # ZOOM CONTROLS
    @property
    def zoom(self):
        """
        Get current zoom level
        min=0 max=255 step=1 default=128
        :return: current value
        :rtype: AdjustableProperty
        """
        return self._zoom

    @zoom.setter
    def zoom(self, value):
        """
        Set zoom
        min=0 max=255 step=1 default=128
        :param value: new value
        :type value: int
        """
        self._zoom.value = value


    # SHARPNESS CONTROLS
    @property
    def sharpness(self):
        """
        Get current sharpness
        min=0 max=255 step=1 default=128
        :return: current value
        :rtype: AdjustableProperty
        """
        return self._sharpness

    @sharpness.setter
    def sharpness(self, value):
        """
        Set sharpness
        min=0 max=255 step=1 default=128
        :param value: new value
        :type value: int
        """
        self._sharpness.value = value

    # SATURATION CONTROLS
    @property
    def saturation(self):
        """
        Get current saturation
        min=0 max=255 step=1 default=128
        :return: current value
        :rtype: AdjustableProperty
        """
        return self._saturation

    @saturation.setter
    def saturation(self, value):
        """
        Set saturation
        min=0 max=255 step=1 default=128
        :param value: new value
        :type value: int
        """
        self._saturation.value = value

    # CONTRAST CONTROLS
    @property
    def contrast(self):
        """
        Get current contrast
        min=0 max=255 step=1 default=128 value=117
        :return: current value
        :rtype: AdjustableProperty
        """
        return self._contrast

    @contrast.setter
    def contrast(self, value):
        """
        Set contrast
        min=0 max=255 step=1 default=128 value=117
        :param value: new value
        :type value: int
        """
        self._contrast.value = value

    # BRIGHTNESS CONTROLS
    @property
    def brightness(self):
        """
        Get current brightness
        min=0 max=255 step=1 default=128 value=117
        :return: current brightness
        :rtype: AdjustableProperty
        """
        return self._brightness

    @brightness.setter
    def brightness(self, value):
        """
        Set brightens
        min=0 max=255 step=1 default=128 value=117
        :param value: new brightness value
        :type value: int
        """
        self._brightness.value = value

    # GAIN CONTROLS
    @property
    def gain(self):
        """
        Get current gain
        min=0 max=255 step=1 default=0 value=255
        :return: current gain
        :rtype: AdjustableProperty
        """
        return self._gain

    @gain.setter
    def gain(self, value):
        """
        Set gain
        min=0 max=255 step=1 default=0 value=255
        :param: current gain
        :type: int
        """
        self._gain.value = value

    # EXPOSURE CONTROLS
    @property
    def exposure(self):
        """
        Get current set exposure from device
        min=3 max=2047 step=1 default=250 value=250 flags=inactive
        :return: current exposure
        :rtype: AutoAdjustableCamProperty
        """
        return self._exposure

    @exposure.setter
    def exposure(self, exposure):
        """
        Sets a new exposure if auto exposure is disabled
        exposure_absolute: min=3 max=2047 step=1 default=250 value=250 flags=inactive
        :param exposure: The new exposure range:
        :type exposure: int
        """
        self._exposure.value = exposure

    # FOCUS CONTROLS
    @property
    def focus(self):
        """
        Get current focus value
        :return: current value
        :rtype: AutoAdjustableCamProperty
        """
        return self._focus

    @focus.setter
    def focus(self, value):
        """
        Sets focus value
        :param value: Focus value
        :type value: int
        """
        self.focus.value = value

    # BACK LIGHT COMPENSATION
    @property
    def backlight_compensation(self):
        """
        Get current state of back light compensation.
        :return: True if activated false if deactivated
        :rtype: AutoProperty
        """
        return self._back_light

    @backlight_compensation.setter
    def backlight_compensation(self, state):
        """
        Activate or deactivate back light compensation
        :param state: True for activate, false for deactivate
        :type state: bool
        """
        self._back_light.auto = state

    # WHITE BALANCE
    @property
    def white_balance(self):
        """
        Get current white balance
        :return: current value
        :rtype: AutoAdjustableCamProperty
        """
        return self._white_balance

    @white_balance.setter
    def white_balance(self, value):
        """
        Sets white balance value
        :param value: Focus value
        :type value: int
        """
        self._white_balance.value = value


if __name__ == "__main__":
    c = C920WebCam(0)
    print c.exposure.auto
    c.exposure.auto = False
    c.exposure.auto = True
    #
    c.exposure = 1000
    print c.exposure
    print c.exposure.min
    print c.exposure.max
    print c.exposure.default

    print c.white_balance.auto
    print c.white_balance

    c.white_balance.auto = False
    print c.white_balance.auto
    c.white_balance = 5000
    print c.white_balance

    print "backlight", c.backlight_compensation
    c.backlight_compensation = True
    print "backlight", c.backlight_compensation

    c.gain = 100
    print c.gain

    c.brightness = 0
    print c.brightness

    c.zoom = 0
    c.sharpness = c.sharpness.default

    c.focus.auto = False
    print c.focus.auto
    c.focus = 0
    c.focus.set_default()

    c.saturation = 255
    c.saturation.set_default()
    c.contrast = 255
    c.contrast.set_default()
