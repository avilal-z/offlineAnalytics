#ifdef PHOXI_FEATURE_DECLARATION
///Set or Get the device Resolution if supported
PhoXiFeature<PhoXiSize> Resolution;
///Set or Get the device CapturingMode if supported
PhoXiFeature<PhoXiCapturingMode> CapturingMode;
///Set or Get the device CapturingSettings
PhoXiFeature<PhoXiCapturingSettings> CapturingSettings;
///Set or Get the device TriggerMode
PhoXiFeature<PhoXiTriggerMode> TriggerMode;
///Set or Get the device Timeout
PhoXiFeature<PhoXiTimeout> Timeout;
///Set or Get the device ProcessingSettings
PhoXiFeature<PhoXiProcessingSettings> ProcessingSettings;
///Set or Get the device OutputSettings
PhoXiFeature<FrameOutputSettings> OutputSettings;
///Get the device SupportedCapturingModes if supported
PhoXiFeature<std::vector<PhoXiCapturingMode>> SupportedCapturingModes;
///Get the device SupportedSinglePatternExposures if supported
PhoXiFeature<std::vector<double>> SupportedSinglePatternExposures;
///Set or Get the device HardwareIdentification
PhoXiFeature<std::string> HardwareIdentification;
///Set or Get the device CoordinatesSettings
PhoXiFeature<PhoXiCoordinatesSettings> CoordinatesSettings;
///Get the device CalibrationSettings
PhoXiFeature<PhoXiCalibrationSettings> CalibrationSettings;

#else
#include "PhoXiCompilerDefines.h"

#define PHOXI_FEATURE_TYPES_BASIC()\
    FEATURE_TYPE(double)\
    FEATURE_TYPE(bool)\
    FEATURE_TYPE(std::vector<double>)

#define PHOXI_FEATURE_TYPES_COMPLEX()\
    FEATURE_TYPE(PhoXiSize)\
    FEATURE_TYPE(std::string)\
    FEATURE_TYPE(PhoXiTriggerMode)\
    FEATURE_TYPE(PhoXiTimeout)\
    FEATURE_TYPE(PhoXiCapturingMode)\
    FEATURE_TYPE(PhoXiCapturingSettings)\
    FEATURE_TYPE(std::vector<PhoXiCapturingMode>)\
    FEATURE_TYPE(PhoXiProcessingSettings)\
    FEATURE_TYPE(FrameOutputSettings)\
    FEATURE_TYPE(PhoXiCoordinatesSettings)\
    FEATURE_TYPE(PhoXiCalibrationSettings)

#define PHOXI_FEATURE_TYPES()\
    PHOXI_FEATURE_TYPES_BASIC()\
    PHOXI_FEATURE_TYPES_COMPLEX()

#define PHOXI_FEATURE_LIST_STATIC_SIZE()\
    PHOXI_FEATURE(Resolution, PhoXiSize, PhoXiSizeInternal)\
    PHOXI_FEATURE(CapturingMode, PhoXiCapturingMode, PhoXiCapturingModeInternal)\
    PHOXI_FEATURE(CapturingSettings, PhoXiCapturingSettings, PhoXiCapturingSettingsInternal)\
    PHOXI_FEATURE(TriggerMode, PhoXiTriggerMode, PhoXiTriggerModeInternal)\
    PHOXI_FEATURE(Timeout, PhoXiTimeout, PhoXiTimeoutInternal)\
    PHOXI_FEATURE(ProcessingSettings, PhoXiProcessingSettings, PhoXiProcessingSettingsInternal)\
    PHOXI_FEATURE(OutputSettings, FrameOutputSettings, FrameOutputSettingsInternal)\
    PHOXI_FEATURE(CoordinatesSettings, PhoXiCoordinatesSettings, PhoXiCoordinatesSettingsInternal)\
    PHOXI_FEATURE(CalibrationSettings, PhoXiCalibrationSettings, PhoXiCalibrationSettingsInternal)\

#define PHOXI_FEATURE_LIST_DYNAMIC_ARRAY()\
    PHOXI_FEATURE(SupportedCapturingModes, std::vector<PhoXiCapturingMode>, PhoXiSupportedCapturingModesInternal)\
    PHOXI_FEATURE(SupportedSinglePatternExposures, std::vector<double>, PhoXiSupportedSinglePatternExposureInternal)\

#define PHOXI_FEATURE_LIST_DYNAMIC_SIZE()\
    PHOXI_FEATURE(HardwareIdentification, std::string, HardwareIdentificationInternal)\

#define PHOXI_FEATURE_LIST()\
    PHOXI_FEATURE_LIST_STATIC_SIZE()\
    PHOXI_FEATURE_LIST_DYNAMIC_ARRAY()\
    PHOXI_FEATURE_LIST_DYNAMIC_SIZE()
#endif //!PHOXI_FEATURE_DECLARATION
