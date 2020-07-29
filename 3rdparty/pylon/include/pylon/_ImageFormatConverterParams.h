//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------

/*!
\file
\brief Interface to image format converter parameters.
*/

//-----------------------------------------------------------------------------
//  This file is generated automatically
//  Do not modify!
//-----------------------------------------------------------------------------

#ifndef BASLER_PYLON_IMAGEFORMATCONVERTERPARAMS_H
#define BASLER_PYLON_IMAGEFORMATCONVERTERPARAMS_H

#pragma once

// common parameter types
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4250 ) // warning C4250: 'Pylon::CXYZParameter': inherits 'Pylon::CParameter::Pylon::CParameter::ZYX' via dominance
#endif

//! The namespace containing the device's control interface and related enumeration types
namespace Basler_ImageFormatConverterParams
{

    //**************************************************************************************************
    // Enumerations
    //**************************************************************************************************

    //! Valid values for InconvertibleEdgeHandling
    enum InconvertibleEdgeHandlingEnums
    {
        InconvertibleEdgeHandling_SetZero,  //!< Rows and columns that can't be converted are set to zero.
        InconvertibleEdgeHandling_Clip,  //!< Rows and columns that can't be converted are removed from the output image.
        InconvertibleEdgeHandling_Extend  //!< Rows and columns that can't be converted are filled by extrapolating image data from neighboring rows and columns.
    };

    //! Valid values for MonoConversionMethod
    enum MonoConversionMethodEnums
    {
        MonoConversionMethod_Gamma,  //!< When converting images, Gamma conversion is used.
        MonoConversionMethod_Truncate  //!< When converting images, image data is shifted.
    };

    //! Valid values for OutputBitAlignment
    enum OutputBitAlignmentEnums
    {
        OutputBitAlignment_LsbAligned,  //!< Image data is aligned with the least significant bit.
        OutputBitAlignment_MsbAligned  //!< The data is aligned at the most significant bit.
    };

    //! Valid values for OutputOrientation
    enum OutputOrientationEnums
    {
        OutputOrientation_Unchanged,  //!< The orientation of the image remains the same.
        OutputOrientation_TopDown,  //!< The first row of the image is located at the start of the image buffer.
        OutputOrientation_BottomUp  //!< The last row of the image is located at the start of the image buffer.
    };



    //**************************************************************************************************
    // Parameter class
    //**************************************************************************************************
    

    //! Interface to image format converter parameters.
    class CImageFormatConverterParams_Params
    {
    //----------------------------------------------------------------------------------------------------------------
    // Implementation
    //----------------------------------------------------------------------------------------------------------------
    protected:
        // If you want to show the following methods in the help file
        // add the string HIDE_CLASS_METHODS to the ENABLED_SECTIONS tag in the doxygen file
        //! \cond HIDE_CLASS_METHODS
        
            //! Constructor
            CImageFormatConverterParams_Params(void);

            //! Destructor
            ~CImageFormatConverterParams_Params(void);

            //! Initializes the references
            void _Initialize(GENAPI_NAMESPACE::INodeMap*);

            //! Return the vendor of the camera
            const char* _GetVendorName(void);

            //! Returns the camera model name
            const char* _GetModelName(void);
        
        //! \endcond

    private:
        class CImageFormatConverterParams_ParamsData;
        CImageFormatConverterParams_ParamsData* m_pData;


    //----------------------------------------------------------------------------------------------------------------
    // References to features
    //----------------------------------------------------------------------------------------------------------------
    public:
        
    //! \name MonoConversion - Parameters for converting monochrome images.
    //@{
    /*!
        \brief Additional shifting value used for converting monochrome images

        Additional shifting value used for converting monochrome images. Only effective if the Mono Conversion Method parameter is set to Truncate. If the parameter value isn't zero, the image data is converted using a lookup table. Shifted values exceeding the maximum output value boundary are set to the maximum allowed value. Negative values are treated as right-shifted values.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& AdditionalLeftShift;
    
    //@}


    //! \name MonoConversion - Parameters for converting monochrome images.
    //@{
    /*!
        \brief Gamma value for converting monochrome images

        Gamma value for converting monochrome images. The image data is converted using a lookup table.
    
        \b Visibility = Beginner

    */
    Pylon::IFloatEx& Gamma;
    
    //@}


    //! \name Root - Image Format Converter parameters.
    //@{
    /*!
        \brief Sets how to handle rows and columns that can't be converted

    
        \b Visibility = Beginner

    */
    Pylon::IEnumParameterT<InconvertibleEdgeHandlingEnums>& InconvertibleEdgeHandling;
    
    //@}


    //! \name MonoConversion - Parameters for converting monochrome images.
    //@{
    /*!
        \brief Sets the conversion method for monochrome images

    
        \b Visibility = Beginner

    */
    Pylon::IEnumParameterT<MonoConversionMethodEnums>& MonoConversionMethod;
    
    //@}


    //! \name Root - Image Format Converter parameters.
    //@{
    /*!
        \brief Sets the alignment of the bits in the target pixel type

        Sets the alignment of the bits in the target pixel type if the target bit depth is greater than the source bit depth, e.g., if you are converting from a 10-bit to a 16-bit format.
    
        \b Visibility = Beginner

    */
    Pylon::IEnumParameterT<OutputBitAlignmentEnums>& OutputBitAlignment;
    
    //@}


    //! \name Root - Image Format Converter parameters.
    //@{
    /*!
        \brief Sets the vertical orientation of the output image in the buffer

    
        \b Visibility = Beginner

    */
    Pylon::IEnumParameterT<OutputOrientationEnums>& OutputOrientation;
    
    //@}


    //! \name Root - Image Format Converter parameters.
    //@{
    /*!
        \brief Number of additional data bytes at the end of each line

        Number of additional data bytes at the end of each line. These bytes are set to zero during the conversion.
    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& OutputPaddingX;
    
    //@}



    private:
        //! \cond HIDE_CLASS_METHODS

            //! not implemented copy constructor
            CImageFormatConverterParams_Params(CImageFormatConverterParams_Params&);

            //! not implemented assignment operator
            CImageFormatConverterParams_Params& operator=(CImageFormatConverterParams_Params&);

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
        class InconvertibleEdgeHandlingEnumParameter : public Pylon::CEnumParameterT<InconvertibleEdgeHandlingEnums>
        {
        public:
            InconvertibleEdgeHandlingEnumParameter()
            {
            }

            virtual ~InconvertibleEdgeHandlingEnumParameter()
            {
            }
        protected:
            virtual const Table_t& GetTable() const
            {
                static const size_t cItemCount = 3;
                static const TableItem_t cItems[cItemCount] =
                {
                    TableItem_t("SetZero", 8),
                    TableItem_t("Clip", 5),
                    TableItem_t("Extend", 7)
                };
                static const Table_t table(cItems, cItemCount);
                return table;
            }
        };


        ///////////////////////////////////////////////////////////////////////////
        //
        class MonoConversionMethodEnumParameter : public Pylon::CEnumParameterT<MonoConversionMethodEnums>
        {
        public:
            MonoConversionMethodEnumParameter()
            {
            }

            virtual ~MonoConversionMethodEnumParameter()
            {
            }
        protected:
            virtual const Table_t& GetTable() const
            {
                static const size_t cItemCount = 2;
                static const TableItem_t cItems[cItemCount] =
                {
                    TableItem_t("Gamma", 6),
                    TableItem_t("Truncate", 9)
                };
                static const Table_t table(cItems, cItemCount);
                return table;
            }
        };


        ///////////////////////////////////////////////////////////////////////////
        //
        class OutputBitAlignmentEnumParameter : public Pylon::CEnumParameterT<OutputBitAlignmentEnums>
        {
        public:
            OutputBitAlignmentEnumParameter()
            {
            }

            virtual ~OutputBitAlignmentEnumParameter()
            {
            }
        protected:
            virtual const Table_t& GetTable() const
            {
                static const size_t cItemCount = 2;
                static const TableItem_t cItems[cItemCount] =
                {
                    TableItem_t("LsbAligned", 11),
                    TableItem_t("MsbAligned", 11)
                };
                static const Table_t table(cItems, cItemCount);
                return table;
            }
        };


        ///////////////////////////////////////////////////////////////////////////
        //
        class OutputOrientationEnumParameter : public Pylon::CEnumParameterT<OutputOrientationEnums>
        {
        public:
            OutputOrientationEnumParameter()
            {
            }

            virtual ~OutputOrientationEnumParameter()
            {
            }
        protected:
            virtual const Table_t& GetTable() const
            {
                static const size_t cItemCount = 3;
                static const TableItem_t cItems[cItemCount] =
                {
                    TableItem_t("Unchanged", 10),
                    TableItem_t("TopDown", 8),
                    TableItem_t("BottomUp", 9)
                };
                static const Table_t table(cItems, cItemCount);
                return table;
            }
        };



    }


    class CImageFormatConverterParams_Params::CImageFormatConverterParams_ParamsData
    {
    public:
        Pylon::CIntegerParameter AdditionalLeftShift;
        Pylon::CFloatParameter Gamma;
        EnumParameterClasses::InconvertibleEdgeHandlingEnumParameter InconvertibleEdgeHandling;
        EnumParameterClasses::MonoConversionMethodEnumParameter MonoConversionMethod;
        EnumParameterClasses::OutputBitAlignmentEnumParameter OutputBitAlignment;
        EnumParameterClasses::OutputOrientationEnumParameter OutputOrientation;
        Pylon::CIntegerParameter OutputPaddingX;
    };


    ///////////////////////////////////////////////////////////////////////////
    //
    inline CImageFormatConverterParams_Params::CImageFormatConverterParams_Params(void)
        : m_pData(new CImageFormatConverterParams_ParamsData())
        , AdditionalLeftShift(m_pData->AdditionalLeftShift)
        , Gamma(m_pData->Gamma)
        , InconvertibleEdgeHandling(m_pData->InconvertibleEdgeHandling)
        , MonoConversionMethod(m_pData->MonoConversionMethod)
        , OutputBitAlignment(m_pData->OutputBitAlignment)
        , OutputOrientation(m_pData->OutputOrientation)
        , OutputPaddingX(m_pData->OutputPaddingX)
    {
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline CImageFormatConverterParams_Params::~CImageFormatConverterParams_Params(void)
    {
        delete m_pData;
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline void CImageFormatConverterParams_Params::_Initialize(GENAPI_NAMESPACE::INodeMap* _Ptr)
    {
        m_pData->AdditionalLeftShift.Attach(_Ptr, "AdditionalLeftShift");
        m_pData->Gamma.Attach(_Ptr, "Gamma");
        m_pData->InconvertibleEdgeHandling.Attach(_Ptr, "InconvertibleEdgeHandling");
        m_pData->MonoConversionMethod.Attach(_Ptr, "MonoConversionMethod");
        m_pData->OutputBitAlignment.Attach(_Ptr, "OutputBitAlignment");
        m_pData->OutputOrientation.Attach(_Ptr, "OutputOrientation");
        m_pData->OutputPaddingX.Attach(_Ptr, "OutputPaddingX");
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline const char* CImageFormatConverterParams_Params::_GetVendorName(void)
    {
        return "Basler";
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline const char* CImageFormatConverterParams_Params::_GetModelName(void)
    {
        return "ImageFormatConverterParams";
    }

    //! \endcond

} // namespace Basler_ImageFormatConverterParams

#ifdef _MSC_VER
#pragma warning( pop )
#endif

#endif // BASLER_PYLON_IMAGEFORMATCONVERTERPARAMS_H
