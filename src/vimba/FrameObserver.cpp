/*=============================================================================
  Copyright (C) 2012 - 2016 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FrameObserver.cpp

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

#include <FrameObserver.h>

namespace AVT {
namespace VmbAPI {
namespace Examples {

//
// This is our callback routine that will be executed on every received frame.
// Triggered by the API.
//
// Parameters:
//  [in]    pFrame          The frame returned from the API
//
void FrameObserver::FrameReceived( const FramePtr pFrame )
{
    bool bQueueDirectly = true;
    VmbFrameStatusType eReceiveStatus;

    if(     0 != receivers(SIGNAL(FrameReceivedSignal(int)) ) 
        &&  VmbErrorSuccess == pFrame->GetReceiveStatus( eReceiveStatus ) )
    {
        // Lock the frame queue
        m_FramesMutex.lock();
        // Add frame to queue
        m_Frames.push( pFrame );
        // Unlock frame queue
        m_FramesMutex.unlock();
        // Emit the frame received signal
        emit FrameReceivedSignal( eReceiveStatus );
        bQueueDirectly = false;
    }

    // If any error occurred we queue the frame without notification
    if( true == bQueueDirectly )
    {
        m_pCamera->QueueFrame( pFrame );
    }
}

//
// After the view has been notified about a new frame it can pick it up.
// It is then removed from the internal queue
//
// Returns:
//  A shared pointer to the latest frame
//
FramePtr FrameObserver::GetFrame()
{
    // Lock the frame queue
    m_FramesMutex.lock();
    // Pop frame from queue
    FramePtr res;
    if( !m_Frames.empty() )
    {
        res = m_Frames.front();
        m_Frames.pop();
    }
    // Unlock frame queue
    m_FramesMutex.unlock();
    return res;
}

//
// Clears the internal (double buffering) frame queue
//
void FrameObserver::ClearFrameQueue()
{
    // Lock the frame queue
    m_FramesMutex.lock();
    // Clear the frame queue and release the memory
    std::queue<FramePtr> empty;
    std::swap( m_Frames, empty );
    // Unlock the frame queue
    m_FramesMutex.unlock();
}

}}} // namespace AVT::VmbAPI::Examples
