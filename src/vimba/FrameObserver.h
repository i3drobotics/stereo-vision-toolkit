/*=============================================================================
  Copyright (C) 2012 - 2016  Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FrameObserver.h

  Description: The frame observer that is used for notifications from VimbaCPP
               regarding the arrival of a newly acquired frame.

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

#ifndef AVT_VMBAPI_EXAMPLES_FRAMEOBSERVER
#define AVT_VMBAPI_EXAMPLES_FRAMEOBSERVER

#include <queue>

#include <QObject>
#include <QMutex>

#include <VimbaCPP/Include/VimbaCPP.h>

namespace AVT {
namespace VmbAPI {
namespace Examples {

class FrameObserver : public QObject, virtual public IFrameObserver
{
    Q_OBJECT

  public:
    //
    // We pass the camera that will deliver the frames to the constructor
    //
    // Parameters:
    //  [in]    pCamera         The camera the frame was queued at
    //
    FrameObserver( CameraPtr pCamera ) : IFrameObserver( pCamera ) {;}
    
    //
    // This is our callback routine that will be executed on every received frame.
    // Triggered by the API.
    //
    // Parameters:
    //  [in]    pFrame          The frame returned from the API
    //
    virtual void FrameReceived( const FramePtr pFrame );

    //
    // After the view has been notified about a new frame it can pick it up.
    // It is then removed from the internal queue
    //
    // Returns:
    //  A shared pointer to the latest frame
    //
    FramePtr GetFrame();

    //
    // Clears the internal (double buffering) frame queue
    //
    void ClearFrameQueue();

  private:
    // Since a Qt signal cannot contain a whole frame
    // the frame observer stores all FramePtr
    std::queue<FramePtr> m_Frames;
    QMutex m_FramesMutex;

signals:
    //
    // The frame received event (Qt signal) that notifies about a new incoming frame
    //
    // Parameters:
    //  [out]   status          The frame receive status
    //
    void FrameReceivedSignal( int status );
};

}}} // namespace AVT::VmbAPI::Examples

#endif
