//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------

/*!
\file
\brief A parameter class containing all parameters as members that are available for ace USB, and GigE
*/

//-----------------------------------------------------------------------------
//  This file is generated automatically
//  Do not modify!
//-----------------------------------------------------------------------------

#ifndef BASLER_PYLON_UNIVERSALCHUNKDATAPARAMS_H
#define BASLER_PYLON_UNIVERSALCHUNKDATAPARAMS_H

#pragma once

// common parameter types
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4250 ) // warning C4250: 'Pylon::CXYZParameter': inherits 'Pylon::CParameter::Pylon::CParameter::ZYX' via dominance
#endif

//! The namespace containing the device's control interface and related enumeration types
namespace Basler_UniversalChunkDataParams
{

    //**************************************************************************************************
    // Enumerations
    //**************************************************************************************************

    //! Valid values for ChunkCounterSelector
    enum ChunkCounterSelectorEnums
    {
        ChunkCounterSelector_Counter1,  //!< Chunk data can be retrieved from counter 1. - Applies to: ace USB
        ChunkCounterSelector_Counter2  //!< Chunk data can be retrieved from counter 2. - Applies to: ace USB
    };

    //! Valid values for ChunkGainSelector
    enum ChunkGainSelectorEnums
    {
        ChunkGainSelector_All  //!< Chunk data can be retrieved from all gain channels. - Applies to: ace USB
    };

    //! Valid values for ChunkPixelFormat
    enum ChunkPixelFormatEnums
    {
        ChunkPixelFormat_Mono8,  //!< Indicates that the pixel data in the acquired image is in the Mono 8 format - Applies to: GigE
        ChunkPixelFormat_Mono8Signed,  //!< Indicates that the pixel data in the acquired image is in the Mono 8 signed format - Applies to: GigE
        ChunkPixelFormat_Mono10,  //!< Indicates that the pixel data in the acquired image is in the Mono 10 format - Applies to: GigE
        ChunkPixelFormat_Mono10Packed,  //!< Indicates that the pixel data in the acquired image is in the Mono 10 Packed format - Applies to: GigE
        ChunkPixelFormat_Mono10p,  //!< Indicates that the pixel data in the acquired image is in the Mono 10p format - Applies to: GigE
        ChunkPixelFormat_Mono12,  //!< Indicates that the pixel data in the acquired image is in the Mono 12 format - Applies to: GigE
        ChunkPixelFormat_Mono12Packed,  //!< Indicates that the pixel data in the acquired image is in the Mono 12 Packed format - Applies to: GigE
        ChunkPixelFormat_Mono16,  //!< Indicates that the pixel data in the acquired image is in the Mono 16 format - Applies to: GigE
        ChunkPixelFormat_BayerGR8,  //!< Indicates that the pixel data in the acquired image is in the Bayer GR 8 format - Applies to: GigE
        ChunkPixelFormat_BayerRG8,  //!< Indicates that the pixel data in the acquired image is in the Bayer RG 8 format - Applies to: GigE
        ChunkPixelFormat_BayerGB8,  //!< Indicates that the pixel data in the acquired image is in the Bayer GB 8 format - Applies to: GigE
        ChunkPixelFormat_BayerBG8,  //!< Indicates that the pixel data in the acquired image is in the Bayer BG 8 format - Applies to: GigE
        ChunkPixelFormat_BayerGR10,  //!< Indicates that the pixel data in the acquired image is in the Bayer GR 10 format - Applies to: GigE
        ChunkPixelFormat_BayerRG10,  //!< Indicates that the pixel data in the acquired image is in the Bayer RG 10 format - Applies to: GigE
        ChunkPixelFormat_BayerGB10,  //!< Indicates that the pixel data in the acquired image is in the Bayer GB 10 format - Applies to: GigE
        ChunkPixelFormat_BayerBG10,  //!< Indicates that the pixel data in the acquired image is in the Bayer BG 10 format - Applies to: GigE
        ChunkPixelFormat_BayerGR12,  //!< Indicates that the pixel data in the acquired image is in the Bayer GR 12 format - Applies to: GigE
        ChunkPixelFormat_BayerRG12,  //!< Indicates that the pixel data in the acquired image is in the Bayer RG 12 format - Applies to: GigE
        ChunkPixelFormat_BayerGB12,  //!< Indicates that the pixel data in the acquired image is in the Bayer GB 12 format - Applies to: GigE
        ChunkPixelFormat_BayerBG12,  //!< Indicates that the pixel data in the acquired image is in the Bayer BG 12 format - Applies to: GigE
        ChunkPixelFormat_BayerGR16,  //!< Indicates that the pixel data in the acquired image is in the Bayer GR 16 format - Applies to: GigE
        ChunkPixelFormat_BayerRG16,  //!< Indicates that the pixel data in the acquired image is in the Bayer RG 16 format - Applies to: GigE
        ChunkPixelFormat_BayerGB16,  //!< Indicates that the pixel data in the acquired image is in the Bayer GB 16 format - Applies to: GigE
        ChunkPixelFormat_BayerBG16,  //!< Indicates that the pixel data in the acquired image is in the Bayer BG 16 format - Applies to: GigE
        ChunkPixelFormat_RGB8Packed,  //!< Indicates that the pixel data in the acquired image is in the RGB 8 Packed format - Applies to: GigE
        ChunkPixelFormat_BGR8Packed,  //!< Indicates that the pixel data in the acquired image is in the BGR 8 Packed format - Applies to: GigE
        ChunkPixelFormat_RGBA8Packed,  //!< Indicates that the pixel data in the acquired image is in the RGBA 8 Packed format - Applies to: GigE
        ChunkPixelFormat_BGRA8Packed,  //!< Indicates that the pixel data in the acquired image is in the BGRA 8 Packed format - Applies to: GigE
        ChunkPixelFormat_RGB10Packed,  //!< Indicates that the pixel data in the acquired image is in the RGB 10 Packed format - Applies to: GigE
        ChunkPixelFormat_BGR10Packed,  //!< Indicates that the pixel data in the acquired image is in the BGR 10 Packed format - Applies to: GigE
        ChunkPixelFormat_RGB12Packed,  //!< Indicates that the pixel data in the acquired image is in the RGB 12 Packed format - Applies to: GigE
        ChunkPixelFormat_BGR12Packed,  //!< Indicates that the pixel data in the acquired image is in the BGR 12 Packed format - Applies to: GigE
        ChunkPixelFormat_RGB10V1Packed,  //!< Indicates that the pixel data in the acquired image is in the RGB 10V1 Packed format - Applies to: GigE
        ChunkPixelFormat_RGB10V2Packed,  //!< Indicates that the pixel data in the acquired image is in the RGB 10V2 Packed format - Applies to: GigE
        ChunkPixelFormat_YUV411Packed,  //!< Indicates that the pixel data in the acquired image is in the YUV 411 Packed format - Applies to: GigE
        ChunkPixelFormat_YUV422Packed,  //!< Indicates that the pixel data in the acquired image is in the YUV 422 Packed format - Applies to: GigE
        ChunkPixelFormat_YUV444Packed,  //!< Indicates that the pixel data in the acquired image is in the YUV 444 Packed format - Applies to: GigE
        ChunkPixelFormat_RGB8Planar,  //!< Indicates that the pixel data in the acquired image is in the RGB 8 Planar format - Applies to: GigE
        ChunkPixelFormat_RGB10Planar,  //!< Indicates that the pixel data in the acquired image is in the RGB 10 Planar format - Applies to: GigE
        ChunkPixelFormat_RGB12Planar,  //!< Indicates that the pixel data in the acquired image is in the RGB 12 Planar format - Applies to: GigE
        ChunkPixelFormat_RGB16Planar,  //!< Indicates that the pixel data in the acquired image is in the RGB 16 Planar format - Applies to: GigE
        ChunkPixelFormat_YUV422_YUYV_Packed,  //!< Indicates that the pixel data in the acquired image is in the YUV 422 (YUYV) Packed format - Applies to: GigE
        ChunkPixelFormat_BayerGB12Packed,  //!< Indicates that the pixel data in the acquired image is in the Bayer GB 12 Packed format - Applies to: GigE
        ChunkPixelFormat_BayerGR12Packed,  //!< Indicates that the pixel data in the acquired image is in the Bayer GR 12 Packed format - Applies to: GigE
        ChunkPixelFormat_BayerRG12Packed,  //!< Indicates that the pixel data in the acquired image is in the Bayer RG 12 Packed format - Applies to: GigE
        ChunkPixelFormat_BayerBG12Packed,  //!< Indicates that the pixel data in the acquired image is in the Bayer BG 12 Packed format - Applies to: GigE
        ChunkPixelFormat_RGB12V1Packed,  //!< Indicates that the pixel data in the acquired image is in RGB 12 Packed  - Applies to: GigE
        ChunkPixelFormat_BayerGB10p,  //!< Indicates that the pixel data in the acquired image is in the Bayer GB 10p format - Applies to: GigE
        ChunkPixelFormat_BayerGR10p,  //!< Indicates that the pixel data in the acquired image is in the Bayer GR 10p format - Applies to: GigE
        ChunkPixelFormat_BayerRG10p,  //!< Indicates that the pixel data in the acquired image is in the Bayer RG 10p format - Applies to: GigE
        ChunkPixelFormat_BayerBG10p  //!< Indicates that the pixel data in the acquired image is in the Bayer BG 10p format - Applies to: GigE
    };



    //**************************************************************************************************
    // Parameter class
    //**************************************************************************************************
    

    //! A parameter class containing all parameters as members that are available for ace USB, and GigE
    class PYLONBASE_API CUniversalChunkDataParams_Params
    {
    //----------------------------------------------------------------------------------------------------------------
    // Implementation
    //----------------------------------------------------------------------------------------------------------------
    protected:
        // If you want to show the following methods in the help file
        // add the string HIDE_CLASS_METHODS to the ENABLED_SECTIONS tag in the doxygen file
        //! \cond HIDE_CLASS_METHODS
        
            //! Constructor
            CUniversalChunkDataParams_Params(void);

            //! Destructor
            ~CUniversalChunkDataParams_Params(void);

            //! Initializes the references
            void _Initialize(GENAPI_NAMESPACE::INodeMap*);

            //! Return the vendor of the camera
            const char* _GetVendorName(void);

            //! Returns the camera model name
            const char* _GetModelName(void);
        
        //! \endcond

    private:
        class CUniversalChunkDataParams_ParamsData;
        CUniversalChunkDataParams_ParamsData* m_pData;


    //----------------------------------------------------------------------------------------------------------------
    // References to features
    //----------------------------------------------------------------------------------------------------------------
    public:
        
    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Sets which counter to retrieve chunk data from - Applies to: ace USB

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<ChunkCounterSelectorEnums>& ChunkCounterSelector;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Value of the selected chunk counter - Applies to: ace USB

    
        \b Visibility = Beginner

        \b Selected by : ChunkCounterSelector

    */
    Pylon::IIntegerEx& ChunkCounterValue;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the maximum possible pixel value in the acquired image - Applies to: GigE

        This value indicates indicates the maximum possible pixel value acquired in the image
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkDynamicRangeMax;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the minimum possible pixel value in the acquired image - Applies to: GigE

        This value indicates the minimum possible pixel value in the acquired image.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkDynamicRangeMin;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Exposure time used to acquire the image - Applies to: ace USB and GigE

    
        \b Visibility = Beginner

    */
    Pylon::IFloatEx& ChunkExposureTime;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Value of the frame trigger counter when the image was acquired - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkFrameTriggerCounter;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Value of the frame trigger ignored counter when the image was acquired - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkFrameTriggerIgnoredCounter;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the value of the frame counter when the image was acquired - Applies to: GigE

        This integer indicates the value of the frame counter when the image was acquired.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkFramecounter;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Value of the frames per trigger counter when the image was acquired - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkFramesPerTriggerCounter;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Gain used to acquire the image - Applies to: ace USB

    
        \b Visibility = Beginner

        \b Selected by : ChunkGainSelector

    */
    Pylon::IFloatEx& ChunkGain;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Gain all setting of the acquired image - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkGainAll;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Sets which gain channel to retrieve chunk data from - Applies to: ace USB

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<ChunkGainSelectorEnums>& ChunkGainSelector;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the height of the area of interest represented in the acquired image - Applies to: GigE

        This value Indicates the height of the area of interest represented in the acquired image.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkHeight;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Number of bits per status - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkInputStatusAtLineTriggerBitsPerLine;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Used to select a certain status - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkInputStatusAtLineTriggerIndex;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Value of the status selected by 'Index' - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkInputStatusAtLineTriggerValue;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief A bit field that indicates the status of all of the camera's input and output lines when the image was acquired - Applies to: ace USB and GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkLineStatusAll;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Value of the line trigger counter when the image was acquired - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkLineTriggerCounter;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Value of the line trigger end to end counter when the image was acquired - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkLineTriggerEndToEndCounter;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Value of the line trigger ignored counter when the image was acquired - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkLineTriggerIgnoredCounter;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the X offset of the area of interest represented in the acquired image - Applies to: GigE

        This value Indicates the X offset of the area of interest represented in the acquired image.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkOffsetX;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the Y offset of the area of interest represented in the acquired image - Applies to: GigE

        This value Indicates the Y offset of the area of interest represented in the acquired image.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkOffsetY;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief CRC checksum of the acquired image - Applies to: ace USB and GigE

        CRC checksum of the acquired image. The checksum is calculated using all of the image data and all of the appended chunks except for the checksum itself.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkPayloadCRC16;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the format of the pixel data in the acquired image - Applies to: GigE

        This enumeration lists the pixel formats that can be indicated by the pixel format chunk.
    
        \b Visibility = Beginner

    */
    Pylon::IEnumParameterT<ChunkPixelFormatEnums>& ChunkPixelFormat;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the sequence set index number related to the acquired image - Applies to: GigE

        This value indicates the sequence set index number related to the acquired image.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkSequenceSetIndex;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Index of the active sequencer set - Applies to: ace USB

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkSequencerSetActive;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Shaft encoder counter at frame trigger - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkShaftEncoderCounter;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the number of bytes of data between the beginning of one line in the acquired image and the beginning of the next line in the acquired image - Applies to: GigE

        This value indicates the number of bytes of data between the beginning of one line in the acquired image and the beginning of the next line in the acquired image.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkStride;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Value of the timestamp when the image was acquired - Applies to: ace USB and GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkTimestamp;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the value of the trigger input counter when the image was acquired - Applies to: GigE

        This integer indicates the value of the trigger input counter when the image was acquired.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkTriggerinputcounter;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief A bit field that indicates the status of all of the camera's virtual input and output lines when the image was acquired - Applies to: GigE

        This value is a bit field that indicates the status of all of the camera's virtual input and output lines when the image was acquired.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkVirtLineStatusAll;
    
    //@}


    //! \name Root - Contains parameters to append chunk data to the image data.
    //@{
    /*!
        \brief Indicates the widtth of the area of interest represented in the acquired image - Applies to: GigE

        This value Indicates the width of the area of interest represented in the acquired image.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ChunkWidth;
    
    //@}



    private:
        //! \cond HIDE_CLASS_METHODS

            //! not implemented copy constructor
            CUniversalChunkDataParams_Params(CUniversalChunkDataParams_Params&);

            //! not implemented assignment operator
            CUniversalChunkDataParams_Params& operator=(CUniversalChunkDataParams_Params&);

        //! \endcond
    };


} // namespace Basler_UniversalChunkDataParams

#ifdef _MSC_VER
#pragma warning( pop )
#endif

#endif // BASLER_PYLON_UNIVERSALCHUNKDATAPARAMS_H
