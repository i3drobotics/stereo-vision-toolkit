/*=============================================================================
  Copyright (C) 2013 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ApiController.cpp

  Description: Implementation file for the ApiController helper class that
               demonstrates how to implement a synchronous single image
               acquisition with VimbaCPP.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#include <sstream>
#include <iostream>

#include "ApiController.h"
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

enum { NUM_FRAMES = 3, };

ApiController::ApiController()
    // Get a reference to the Vimba singleton
    : m_system ( VimbaSystem::GetInstance() )
{
}

ApiController::~ApiController()
{
}

//
// Starts the Vimba API and loads all transport layers
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::StartUp()
{
    return m_system.Startup();
}

//
// Shuts down the API
//
void ApiController::ShutDown()
{
    // Release Vimba
    m_system.Shutdown();
}

VmbErrorType ApiController::OpenCamera( const std::string &rStrCameraID)
{
    CameraPtr pCamera;
    VmbErrorType res = m_system.OpenCameraByID( rStrCameraID.c_str(), VmbAccessModeFull, pCamera );
    return res;
}

VmbErrorType ApiController::GetCamera( const std::string &rStrCameraID, CameraPtr pCamera)
{
    VmbErrorType res = m_system.GetCameraByID(rStrCameraID.c_str(), pCamera);
    return res;
}

VmbErrorType ApiController::AcquireCameraImage( const std::string &rStrCameraID, FramePtr &rpFrame )
{
    CameraPtr pCamera;
    VmbErrorType res = GetCamera(rStrCameraID,pCamera);
    if (res == VmbErrorSuccess){
        res = pCamera->AcquireSingleImage( rpFrame, 5000 );
    }
    return res;
}

//
// Opens the given camera
// Sets the maximum possible Ethernet packet size
// Adjusts the image format
// Calls the API convenience function to start single image acquisition
// Closes the camera in case of failure
//
// Parameters:
//  [in]    rStrCameraID        The ID of the camera to work on
//  [out]   rpFrame             The frame that will be filled. Does not need to be initialized.
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::AcquireSingleImage( const std::string &rStrCameraID, FramePtr &rpFrame )
{
    // Open the desired camera by its ID
    // The currently streaming camera
    CameraPtr pCamera;
    VmbErrorType res = m_system.OpenCameraByID( rStrCameraID.c_str(), VmbAccessModeFull, pCamera );
    if ( VmbErrorSuccess == res )
    {
        FeaturePtr pFormatFeature;
        // Set pixel format. For the sake of simplicity we only support Mono and BGR in this example.
        res = pCamera->GetFeatureByName( "PixelFormat", pFormatFeature );
        if ( VmbErrorSuccess == res )
        {
            // Try to set BGR
            res = pFormatFeature->SetValue( VmbPixelFormatMono8 );

            if ( VmbErrorSuccess == res )
            {
                // Acquire
                res = pCamera->AcquireSingleImage( rpFrame, 5000 );
            }
        }

        pCamera->Close();
    }

    return res;
}

//
// Gets all cameras known to Vimba
//
// Returns:
//  A vector of camera shared pointers
//
CameraPtrVector ApiController::GetCameraList()
{
    CameraPtrVector cameras;
    // Get all known cameras
    if ( VmbErrorSuccess == m_system.GetCameras( cameras ))
    {
        // And return them
        return cameras;
    }
    return CameraPtrVector();
}

//
// Translates Vimba error codes to readable error messages
//
// Parameters:
//  [in]    eErr        The error code to be converted to string
//
// Returns:
//  A descriptive string representation of the error code
//
/*
std::string ApiController::ErrorCodeToMessage( VmbErrorType eErr ) const
{
    return AVT::VmbAPI::Examples::ErrorCodeToMessage( eErr );
}
*/

//
// Gets the version of the Vimba API
//
// Returns:
//  The version as string
//
std::string ApiController::GetVersion() const
{
    std::ostringstream os;
    os<<m_system;
    return os.str();
}

/** read an integer feature from camera.
*/
VmbErrorType ApiController::GetFeatureIntValue( const CameraPtr &camera, const std::string &featureName, VmbInt64_t & value )
{
    if( SP_ISNULL( camera ) )
    {
        return VmbErrorBadParameter;
    }
    FeaturePtr      pFeature;
    VmbErrorType    result;
    result = SP_ACCESS( camera )->GetFeatureByName( featureName.c_str(), pFeature );
    if( VmbErrorSuccess == result )
    {
        result = SP_ACCESS( pFeature )->GetValue( value );
    }
    return result;
}

/** write an integer feature from camera.
*/
VmbErrorType ApiController::SetFeatureIntValue( const CameraPtr &camera, const std::string &featureName, VmbInt64_t value )
{
    if( SP_ISNULL( camera ) )
    {
        return VmbErrorBadParameter;
    }
    FeaturePtr      pFeature;
    VmbErrorType    result;
    result = SP_ACCESS( camera )->GetFeatureByName( featureName.c_str(), pFeature );
    if( VmbErrorSuccess == result )
    {
        result = SP_ACCESS( pFeature )->SetValue( value );
    }
    return result;
}

}}} // namespace AVT::VmbAPI::Examples
