#pragma once

// This file contains shared classes and enums 
namespace arm_mazzolini 
{
    enum class ErrorType 
    {
        TARGET_TOO_FAR,
        EXCLUSION_ZONE,
        TARGET_EMPTY,
        // Che aggiungo????
    };

    enum class MaskType
    {
        HSV_red,
        ExG_threshold,
        ExG_Otsu,
        ExGR,
    };

    enum class DetectorType
    {
        HSV_red,
        ExG_threshold,
        ExG_Otsu,
        ExGR,
        YOLO
    };
}