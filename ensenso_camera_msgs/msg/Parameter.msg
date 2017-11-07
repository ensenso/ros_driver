# Possible parameter keys. Most of the parameters are mapped to parameter nodes
# in the NxLib. In that case, the strings are derived from the names of these
# nodes.
# The comment above each key specifies the data type that is used for this
# parameter (also see the value fields below).
# See the NxLib documentation for information on what the different parameters do.

# Bool
string AUTO_EXPOSURE      = AutoExposure
# Bool
string AUTO_GAIN          = AutoGain
# Float
string BINNING            = Binning
# Float
string EXPOSURE           = Exposure
# Float
string FLEX_VIEW          = FlexView
# Bool
string FRONT_LIGHT        = FrontLight
# Float
string GAIN               = Gain
# Bool
string GAIN_BOOST         = GainBoost
# Bool
string HARDWARE_GAMMA     = HardwareGamma
# Float
string MAX_GAIN           = MaxGain
# Float
string PIXEL_CLOCK        = PixelClock
# Bool
string PROJECTOR          = Projector
# RegionOfInterest
string REGION_OF_INTEREST = RegionOfInterest
# Float
string TARGET_BRIGHTNESS  = TargetBrightness
# Float
string TRIGGER_DELAY      = TriggerDelay
# String
string TRIGGER_MODE       = TriggerMode

# The name of the parameter.
string key

# Values of different types. Only one of them is used, depending on the
# parameter that is specified by the key.
# Integer parameters will use the float value and round it.
bool bool_value
float64 float_value
string string_value
RegionOfInterest region_of_interest_value