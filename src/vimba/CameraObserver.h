/*=============================================================================
  Copyright (C) 2012 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        CameraObserver.h

  Description: The camera observer that is used for notifications from VimbaCPP
               regarding a change in the camera list.

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

#ifndef AVT_VMBAPI_EXAMPLES_CAMERAOBSERVER
#define AVT_VMBAPI_EXAMPLES_CAMERAOBSERVER

#include <QObject>

#include <VimbaCPP/Include/VimbaCPP.h>

namespace AVT {
namespace VmbAPI {
namespace Examples {

class CameraObserver : public QObject, public ICameraListObserver
{
  Q_OBJECT

  public:
    //
    // This is our callback routine that will be executed every time a camera was plugged in or out
    //
    // Parameters:
    //  [in]    pCam            The camera that triggered the callback
    //  [in]    reason          The reason why the callback was triggered
    //
    virtual void CameraListChanged( CameraPtr pCamera, UpdateTriggerType reason );

  signals:
    //
    // The camera list changed event (Qt signal) that notifies about a camera change and its reason
    //
    // Parameters:
    //  [out]    reason          The reason why this event was fired
    //
    void CameraListChangedSignal( int reason );
};

}}} // namespace AVT::VmbAPI::Examples

#endif
