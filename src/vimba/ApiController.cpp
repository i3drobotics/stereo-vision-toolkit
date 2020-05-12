/*=============================================================================
  Copyright (C) 2012 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ApiController.cpp

  Description: Implementation file for the ApiController helper class that
               demonstrates how to implement an asynchronous, continuous image
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

#include <ApiController.h>
#include <sstream>
#include <iostream>
#include "Common/StreamSystemInfo.h"
#include "Common/ErrorCodeToMessage.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

enum    { NUM_FRAMES=3, };

ApiController::ApiController()
    // Get a reference to the Vimba singleton
    : m_system( VimbaSystem::GetInstance() )
{
}

ApiController::~ApiController()
{
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
std::string ApiController::ErrorCodeToMessage( VmbErrorType eErr ) const
{
    return AVT::VmbAPI::Examples::ErrorCodeToMessage( eErr );
}

//
// Starts the Vimba API and loads all transport layers
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::StartUp()
{
    VmbErrorType res;

    // Start Vimba
    res = m_system.Startup();
    if( VmbErrorSuccess == res )
    {
        // This will be wrapped in a shared_ptr so we don't delete it
        SP_SET( m_pCameraObserver , new CameraObserver() );
        // Register an observer whose callback routine gets triggered whenever a camera is plugged in or out
        res = m_system.RegisterCameraListObserver( m_pCameraObserver );
    }

    return res;
}

//
// Shuts down the API
//
void ApiController::ShutDown()
{
    // Release Vimba
    m_system.Shutdown();
}
/*** helper function to set image size to a value that is dividable by modulo 2 and a multiple of the increment.
\note this is needed because VimbaImageTransform does not support odd values for some input formats
*/
inline VmbErrorType SetValueIntMod2( const CameraPtr &camera, const std::string &featureName, VmbInt64_t &storage )
{
    VmbErrorType    res;
    FeaturePtr      pFeature;
    VmbInt64_t      minValue = 0;
    VmbInt64_t      maxValue = 0;
    VmbInt64_t      incrementValue = 0;

    res = SP_ACCESS( camera )->GetFeatureByName( featureName.c_str(), pFeature );
    if( VmbErrorSuccess != res )
    {
        return res;
    }

    res = SP_ACCESS( pFeature )->GetRange( minValue, maxValue );
    if( VmbErrorSuccess != res )
    {
        return res;
    }

    res = SP_ACCESS( pFeature )->GetIncrement( incrementValue);
    if( VmbErrorSuccess != res)
    {
        return res;
    }

    maxValue = maxValue - ( maxValue % incrementValue );
    if( maxValue % 2 != 0)
    {
        maxValue -= incrementValue;
    }

    res = SP_ACCESS( pFeature )->SetValue( maxValue );
    if( VmbErrorSuccess != res )
    {
        return res;
    }

    storage = maxValue;
    return res;
}

//
// Opens the given camera
// Sets the maximum possible Ethernet packet size
// Adjusts the image format
// Sets up the observer that will be notified on every incoming frame
// Calls the API convenience function to start image acquisition
// Closes the camera in case of failure
//
// Parameters:
//  [in]    rStrCameraID    The ID of the camera to open as reported by Vimba
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::StartContinuousImageAcquisition( const std::string &rStrCameraID )
{
    // Open the desired camera by its ID
    VmbErrorType res = m_system.OpenCameraByID( rStrCameraID.c_str(), VmbAccessModeFull, m_pCamera );
    if( VmbErrorSuccess == res )
    {
        // Set the GeV packet size to the highest possible value
        // (In this example we do not test whether this cam actually is a GigE cam)
        FeaturePtr pCommandFeature;
        if( VmbErrorSuccess == SP_ACCESS( m_pCamera )->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ) )
        {
            if( VmbErrorSuccess == SP_ACCESS( pCommandFeature )->RunCommand() )
            {
                bool bIsCommandDone = false;
                do
                {
                    if( VmbErrorSuccess != SP_ACCESS( pCommandFeature )->IsCommandDone( bIsCommandDone ) )
                    {
                        break;
                    }
                } while( false == bIsCommandDone );
            }
        }
        res = SetValueIntMod2( m_pCamera,"Width", m_nWidth );
        if( VmbErrorSuccess == res )
        {
            res = SetValueIntMod2( m_pCamera, "Height", m_nHeight );
            if( VmbErrorSuccess == res )
            {
                // Store currently selected image format
                FeaturePtr pFormatFeature;
                res = SP_ACCESS( m_pCamera )->GetFeatureByName( "PixelFormat", pFormatFeature );
                if( VmbErrorSuccess == res )
                {
                    res = SP_ACCESS( pFormatFeature )->GetValue( m_nPixelFormat );
                    if ( VmbErrorSuccess == res )
                    {
                        // Create a frame observer for this camera (This will be wrapped in a shared_ptr so we don't delete it)
                        SP_SET( m_pFrameObserver , new FrameObserver( m_pCamera ) );
                        // Start streaming
                        res = SP_ACCESS( m_pCamera )->StartContinuousImageAcquisition( NUM_FRAMES,  m_pFrameObserver );
                    }
                }
            }
        }
        if ( VmbErrorSuccess != res )
        {
            // If anything fails after opening the camera we close it
            SP_ACCESS( m_pCamera )->Close();
        }
    }

    return res;
}

//
// Calls the API convenience function to stop image acquisition
// Closes the camera
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::StopContinuousImageAcquisition()
{
    // Stop streaming
    SP_ACCESS( m_pCamera )->StopContinuousImageAcquisition();

    // Close camera
    return  m_pCamera->Close();
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
    if( VmbErrorSuccess == m_system.GetCameras( cameras ) )
    {
        // And return them
        return cameras;
    }
    return CameraPtrVector();
}

//
// Gets the width of a frame
//
// Returns:
//  The width as integer
//
int ApiController::GetWidth() const
{
    return static_cast<int>(m_nWidth);
}

//
// Gets the height of a frame
//
// Returns:
//  The height as integer
//
int ApiController::GetHeight() const
{
    return static_cast<int>(m_nHeight);
}

//
// Gets the pixel format of a frame
//
// Returns:
//  The pixel format as enum
//
VmbPixelFormatType ApiController::GetPixelFormat() const
{
    return static_cast<VmbPixelFormatType>(m_nPixelFormat);
}

//
// Gets the oldest frame that has not been picked up yet
//
// Returns:
//  A frame shared pointer
//
FramePtr ApiController::GetFrame()
{
    return SP_DYN_CAST( m_pFrameObserver, FrameObserver )->GetFrame();
}

//
// Clears all remaining frames that have not been picked up
//
void ApiController::ClearFrameQueue()
{
    SP_DYN_CAST( m_pFrameObserver,FrameObserver )->ClearFrameQueue();
}

//
// Queues a given frame to be filled by the API
//
// Parameters:
//  [in]    pFrame          The frame to queue
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::QueueFrame( FramePtr pFrame )
{
    return SP_ACCESS( m_pCamera )->QueueFrame( pFrame );
}

//
// Returns the camera observer as QObject pointer to connect their signals to the view's slots
//
QObject* ApiController::GetCameraObserver()
{
    return SP_DYN_CAST( m_pCameraObserver, CameraObserver ).get();
}

//
// Returns the frame observer as QObject pointer to connect their signals to the view's slots
//
QObject* ApiController::GetFrameObserver()
{
    return SP_DYN_CAST( m_pFrameObserver, FrameObserver ).get();
}

//
// Gets the version of the Vimba API
//
// Returns:
//  The version as string
//
std::string ApiController::GetVersion() const
{
    std::ostringstream os;
    os << m_system;
    return os.str();
}

}}} // namespace AVT::VmbAPI::Examples
