//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2018-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------

/*!
\file
\brief Interface to video writer parameters.
*/

//-----------------------------------------------------------------------------
//  This file is generated automatically
//  Do not modify!
//-----------------------------------------------------------------------------

#ifndef BASLER_PYLON_VIDEOWRITERPARAMS_H
#define BASLER_PYLON_VIDEOWRITERPARAMS_H

#pragma once

// common parameter types
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4250 ) // warning C4250: 'Pylon::CXYZParameter': inherits 'Pylon::CParameter::Pylon::CParameter::ZYX' via dominance
#endif

//! The namespace containing the device's control interface and related enumeration types
namespace Basler_VideoWriterParams
{

    //**************************************************************************************************
    // Enumerations
    //**************************************************************************************************

    //! Valid values for CompressionMode
    enum CompressionModeEnums
    {
        CompressionMode_Bitrate,  //!< The resulting stream has a constant bit rate.
        CompressionMode_Quality  //!< The resulting stream has a constant quality.
    };



    //**************************************************************************************************
    // Parameter class
    //**************************************************************************************************
    

    //! Interface to video writer parameters.
    class CVideoWriterParams_Params
    {
    //----------------------------------------------------------------------------------------------------------------
    // Implementation
    //----------------------------------------------------------------------------------------------------------------
    protected:
        // If you want to show the following methods in the help file
        // add the string HIDE_CLASS_METHODS to the ENABLED_SECTIONS tag in the doxygen file
        //! \cond HIDE_CLASS_METHODS
        
            //! Constructor
            CVideoWriterParams_Params(void);

            //! Destructor
            ~CVideoWriterParams_Params(void);

            //! Initializes the references
            void _Initialize(GENAPI_NAMESPACE::INodeMap*);

            //! Return the vendor of the camera
            const char* _GetVendorName(void);

            //! Returns the camera model name
            const char* _GetModelName(void);
        
        //! \endcond

    private:
        class CVideoWriterParams_ParamsData;
        CVideoWriterParams_ParamsData* m_pData;


    //----------------------------------------------------------------------------------------------------------------
    // References to features
    //----------------------------------------------------------------------------------------------------------------
    public:
        
    //! \name RecordingOptions - Contains parameters for configuring the recording.
    //@{
    /*!
        \brief Bit rate of the resulting compressed stream

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& Bitrate;
    
    //@}


    //! \name Statistics - Contains parameters with statistical information about the recording.
    //@{
    /*!
        \brief Bytes written to file since starting the recording

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& BytesWritten;
    
    //@}


    //! \name RecordingOptions - Contains parameters for configuring the recording.
    //@{
    /*!
        \brief Sets the compression mode

        Sets the compression mode. You can choose whether to keep the bit rate or the quality of the resulting video stream constant.
    
        \b Visibility = Beginner

    */
    Pylon::IEnumParameterT<CompressionModeEnums>& CompressionMode;
    
    //@}


    //! \name Statistics - Contains parameters with statistical information about the recording.
    //@{
    /*!
        \brief Number of frames written since starting the recording

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& FrameCount;
    
    //@}


    //! \name ImageParameters - Contains parameters for configuring the image.
    //@{
    /*!
        \brief Height of the image (in pixels)

    
        \b Visibility = Invisible

    */
    Pylon::IIntegerEx& Height;
    
    //@}


    //! \name RecordingOptions - Contains parameters for configuring the recording.
    //@{
    /*!
        \brief Frame rate (in Hertz) of the video to be recorded

    
        \b Visibility = Beginner

    */
    Pylon::IFloatEx& PlaybackFrameRate;
    
    //@}


    //! \name RecordingOptions - Contains parameters for configuring the recording.
    //@{
    /*!
        \brief Quality of the resulting compressed stream

        Quality of the resulting compressed stream. The quality has a direct influence on the resulting bit rate. The optimal bit rate is calculated based on the input values height, width, and playback frame rate (WIDTH * HEIGHT * PLAYBACKFRAMERATE * 0.25). This is then normalized to the quality value range 1-100, where 100 corresponds to the optimum bit rate and 1 to the lowest bit rate.
    
        \b Visibility = Beginner

    */
    Pylon::IFloatEx& Quality;
    
    //@}


    //! \name RecordingOptions - Contains parameters for configuring the recording.
    //@{
    /*!
        \brief Number of threads used for recording the video

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& ThreadCount;
    
    //@}


    //! \name ImageParameters - Contains parameters for configuring the image.
    //@{
    /*!
        \brief Width of the image (in pixels)

    
        \b Visibility = Invisible

    */
    Pylon::IIntegerEx& Width;
    
    //@}



    private:
        //! \cond HIDE_CLASS_METHODS

            //! not implemented copy constructor
            CVideoWriterParams_Params(CVideoWriterParams_Params&);

            //! not implemented assignment operator
            CVideoWriterParams_Params& operator=(CVideoWriterParams_Params&);

        //! \endcond
    };



    //**************************************************************************************************
    // Parameter class implementation
    //**************************************************************************************************

    //! \cond HIDE_CLASS_METHODS




    ///////////////////////////////////////////////////////////////////////////
    //
    namespace EnumParameterClasses
    {
        ///////////////////////////////////////////////////////////////////////////
        //
        class CompressionModeEnumParameter : public Pylon::CEnumParameterT<CompressionModeEnums>
        {
        public:
            CompressionModeEnumParameter()
            {
            }

            virtual ~CompressionModeEnumParameter()
            {
            }
        protected:
            virtual const Table_t& GetTable() const
            {
                static const size_t cItemCount = 2;
                static const TableItem_t cItems[cItemCount] =
                {
                    TableItem_t("Bitrate", 8),
                    TableItem_t("Quality", 8)
                };
                static const Table_t table(cItems, cItemCount);
                return table;
            }
        };



    }


    class CVideoWriterParams_Params::CVideoWriterParams_ParamsData
    {
    public:
        Pylon::CIntegerParameter Bitrate;
        Pylon::CIntegerParameter BytesWritten;
        EnumParameterClasses::CompressionModeEnumParameter CompressionMode;
        Pylon::CIntegerParameter FrameCount;
        Pylon::CIntegerParameter Height;
        Pylon::CFloatParameter PlaybackFrameRate;
        Pylon::CFloatParameter Quality;
        Pylon::CIntegerParameter ThreadCount;
        Pylon::CIntegerParameter Width;
    };


    ///////////////////////////////////////////////////////////////////////////
    //
    inline CVideoWriterParams_Params::CVideoWriterParams_Params(void)
        : m_pData(new CVideoWriterParams_ParamsData())
        , Bitrate(m_pData->Bitrate)
        , BytesWritten(m_pData->BytesWritten)
        , CompressionMode(m_pData->CompressionMode)
        , FrameCount(m_pData->FrameCount)
        , Height(m_pData->Height)
        , PlaybackFrameRate(m_pData->PlaybackFrameRate)
        , Quality(m_pData->Quality)
        , ThreadCount(m_pData->ThreadCount)
        , Width(m_pData->Width)
    {
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline CVideoWriterParams_Params::~CVideoWriterParams_Params(void)
    {
        delete m_pData;
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline void CVideoWriterParams_Params::_Initialize(GENAPI_NAMESPACE::INodeMap* _Ptr)
    {
        m_pData->Bitrate.Attach(_Ptr, "Bitrate");
        m_pData->BytesWritten.Attach(_Ptr, "BytesWritten");
        m_pData->CompressionMode.Attach(_Ptr, "CompressionMode");
        m_pData->FrameCount.Attach(_Ptr, "FrameCount");
        m_pData->Height.Attach(_Ptr, "Height");
        m_pData->PlaybackFrameRate.Attach(_Ptr, "PlaybackFrameRate");
        m_pData->Quality.Attach(_Ptr, "Quality");
        m_pData->ThreadCount.Attach(_Ptr, "ThreadCount");
        m_pData->Width.Attach(_Ptr, "Width");
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline const char* CVideoWriterParams_Params::_GetVendorName(void)
    {
        return "Basler";
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline const char* CVideoWriterParams_Params::_GetModelName(void)
    {
        return "VideoWriterParams";
    }

    //! \endcond

} // namespace Basler_VideoWriterParams

#ifdef _MSC_VER
#pragma warning( pop )
#endif

#endif // BASLER_PYLON_VIDEOWRITERPARAMS_H
