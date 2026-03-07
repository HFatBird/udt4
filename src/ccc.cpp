/*****************************************************************************
Copyright (c) 2001 - 2011, The Board of Trustees of the University of Illinois.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the
  above copyright notice, this list of conditions
  and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the University of Illinois
  nor the names of its contributors may be used to
  endorse or promote products derived from this
  software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/

/*****************************************************************************
written by
   Yunhong Gu, last updated 02/21/2013
*****************************************************************************/


#include "core.h"
#include "ccc.h"
#include <cmath>
#include <cstdlib>
#include <cstring>

CCC::CCC():
m_iSYNInterval(CUDT::m_iSYNInterval),
m_dPktSndPeriod(1.0),
m_dCWndSize(16.0),
m_iBandwidth(),
m_dMaxCWndSize(),
m_iMSS(),
m_iSndCurrSeqNo(),
m_iRcvRate(),
m_iRTT(),
m_pcParam(NULL),
m_iPSize(0),
m_UDT(),
m_iACKPeriod(0),
m_iACKInterval(0),
m_bUserDefinedRTO(false),
m_iRTO(-1),
m_PerfInfo()
{
}

CCC::~CCC()
{
   delete [] m_pcParam;
}

void CCC::setACKTimer(int msINT)
{
   m_iACKPeriod = msINT > m_iSYNInterval ? m_iSYNInterval : msINT;
}

void CCC::setACKInterval(int pktINT)
{
   m_iACKInterval = pktINT;
}

void CCC::setRTO(int usRTO)
{
   m_bUserDefinedRTO = true;
   m_iRTO = usRTO;
}

void CCC::sendCustomMsg(CPacket& pkt) const
{
   CUDT* u = CUDT::getUDTHandle(m_UDT);

   if (NULL != u)
   {
      pkt.m_iID = u->m_PeerID;
      u->m_pSndQueue->sendto(u->m_pPeerAddr, pkt);
   }
}

const CPerfMon* CCC::getPerfInfo()
{
   try
   {
      CUDT* u = CUDT::getUDTHandle(m_UDT);
      if (NULL != u)
         u->sample(&m_PerfInfo, false);
   }
   catch (...)
   {
      return NULL;
   }

   return &m_PerfInfo;
}

void CCC::setMSS(int mss)
{
   m_iMSS = mss;
}

void CCC::setBandwidth(int bw)
{
   m_iBandwidth = bw;
}

void CCC::setSndCurrSeqNo(int32_t seqno)
{
   m_iSndCurrSeqNo = seqno;
}

void CCC::setRcvRate(int rcvrate)
{
   m_iRcvRate = rcvrate;
}

void CCC::setMaxCWndSize(int cwnd)
{
   m_dMaxCWndSize = cwnd;
}

void CCC::setRTT(int rtt)
{
   m_iRTT = rtt;
}

void CCC::setUserParam(const char* param, int size)
{
   delete [] m_pcParam;
   m_pcParam = new char[size];
   memcpy(m_pcParam, param, size);
   m_iPSize = size;
}

//
CUDTCC::CUDTCC():
m_iRCInterval(),
m_LastRCTime(),
m_bSlowStart(),
m_iLastAck(),
m_bLoss(),
m_iLastDecSeq(),
m_dLastDecPeriod(),
m_iNAKCount(),
m_iDecRandom(),
m_iAvgNAKNum(),
m_iDecCount()
{
}

void CUDTCC::init()
{
   m_iRCInterval = m_iSYNInterval;
   m_LastRCTime = CTimer::getTime();
   setACKTimer(m_iRCInterval);

   m_bSlowStart = true;
   m_iLastAck = m_iSndCurrSeqNo;
   m_bLoss = false;
   m_iLastDecSeq = CSeqNo::decseq(m_iLastAck);
   m_dLastDecPeriod = 1;
   m_iAvgNAKNum = 0;
   m_iNAKCount = 0;
   m_iDecRandom = 1;

   m_dCWndSize = 16;
   m_dPktSndPeriod = 1;
}

void CUDTCC::onACK(int32_t ack)
{
   int64_t B = 0;
   double inc = 0;
   // Note: 1/24/2012
   // The minimum increase parameter is increased from "1.0 / m_iMSS" to 0.01
   // because the original was too small and caused sending rate to stay at low level
   // for long time.
   const double min_inc = 0.01;

   uint64_t currtime = CTimer::getTime();
   if (currtime - m_LastRCTime < (uint64_t)m_iRCInterval)
      return;

   m_LastRCTime = currtime;

   if (m_bSlowStart)
   {
      m_dCWndSize += CSeqNo::seqlen(m_iLastAck, ack);
      m_iLastAck = ack;

      if (m_dCWndSize > m_dMaxCWndSize)
      {
         m_bSlowStart = false;
         if (m_iRcvRate > 0)
            m_dPktSndPeriod = 1000000.0 / m_iRcvRate;
         else
            m_dPktSndPeriod = (m_iRTT + m_iRCInterval) / m_dCWndSize;
      }
   }
   else
      m_dCWndSize = m_iRcvRate / 1000000.0 * (m_iRTT + m_iRCInterval) + 16;

   // During Slow Start, no rate increase
   if (m_bSlowStart)
      return;

   if (m_bLoss)
   {
      m_bLoss = false;
      return;
   }

   B = (int64_t)(m_iBandwidth - 1000000.0 / m_dPktSndPeriod);
   if ((m_dPktSndPeriod > m_dLastDecPeriod) && ((m_iBandwidth / 9) < B))
      B = m_iBandwidth / 9;
   if (B <= 0)
      inc = min_inc;
   else
   {
      // inc = max(10 ^ ceil(log10( B * MSS * 8 ) * Beta / MSS, 1/MSS)
      // Beta = 1.5 * 10^(-6)

      inc = pow(10.0, ceil(log10(B * m_iMSS * 8.0))) * 0.0000015 / m_iMSS;

      if (inc < min_inc)
         inc = min_inc;
   }

   m_dPktSndPeriod = (m_dPktSndPeriod * m_iRCInterval) / (m_dPktSndPeriod * inc + m_iRCInterval);
}

void CUDTCC::onLoss(const int32_t* losslist, int)
{
   //Slow Start stopped, if it hasn't yet
   if (m_bSlowStart)
   {
      m_bSlowStart = false;
      if (m_iRcvRate > 0)
      {
         // Set the sending rate to the receiving rate.
         m_dPktSndPeriod = 1000000.0 / m_iRcvRate;
         return;
      }
      // If no receiving rate is observed, we have to compute the sending
      // rate according to the current window size, and decrease it
      // using the method below.
      // Packet send period is in microseconds per packet. Keep the same unit
      // conversion used in onACK(): one RTT worth of time divided by packets
      // in flight, not the inverse.
      m_dPktSndPeriod = (m_iRTT + m_iRCInterval) / m_dCWndSize;
   }

   m_bLoss = true;

   if (CSeqNo::seqcmp(losslist[0] & 0x7FFFFFFF, m_iLastDecSeq) > 0)
   {
      m_dLastDecPeriod = m_dPktSndPeriod;
      m_dPktSndPeriod = ceil(m_dPktSndPeriod * 1.125);

      m_iAvgNAKNum = (int)ceil(m_iAvgNAKNum * 0.875 + m_iNAKCount * 0.125);
      m_iNAKCount = 1;
      m_iDecCount = 1;

      m_iLastDecSeq = m_iSndCurrSeqNo;

      // remove global synchronization using randomization
      srand(m_iLastDecSeq);
      m_iDecRandom = (int)ceil(m_iAvgNAKNum * (double(rand()) / RAND_MAX));
      if (m_iDecRandom < 1)
         m_iDecRandom = 1;
   }
   else if ((m_iDecCount ++ < 5) && (0 == (++ m_iNAKCount % m_iDecRandom)))
   {
      // 0.875^5 = 0.51, rate should not be decreased by more than half within a congestion period
      m_dPktSndPeriod = ceil(m_dPktSndPeriod * 1.125);
      m_iLastDecSeq = m_iSndCurrSeqNo;
   }
}

void CUDTCC::onTimeout()
{
   if (m_bSlowStart)
   {
      m_bSlowStart = false;
      if (m_iRcvRate > 0)
         m_dPktSndPeriod = 1000000.0 / m_iRcvRate;
      else
         // Packet send period is in microseconds per packet.
         m_dPktSndPeriod = (m_iRTT + m_iRCInterval) / m_dCWndSize;
   }
   else
   {
      /*
      m_dLastDecPeriod = m_dPktSndPeriod;
      m_dPktSndPeriod = ceil(m_dPktSndPeriod * 2);
      m_iLastDecSeq = m_iLastAck;
      */
   }
}

//
CBBRCC::CBBRCC():
m_BBRVariant(BBR_V1),
m_BBRMode(BBR_STARTUP),
m_LastUpdateTime(),
m_LastRoundStart(),
m_MinRTTStamp(),
m_ProbeRTTDoneStamp(),
m_dBtlBw(),
m_iMinRTT(),
m_iFullBwCount(),
m_iCycleIndex(),
m_iLossEvents(),
m_iAckEvents(),
m_dLossEWMA(),
m_bFilledPipe(),
m_bProbeRTTDone()
{
}

CBBRCC::CBBRCC(BBRVariant variant):
m_BBRVariant(variant),
m_BBRMode(BBR_STARTUP),
m_LastUpdateTime(),
m_LastRoundStart(),
m_MinRTTStamp(),
m_ProbeRTTDoneStamp(),
m_dBtlBw(),
m_iMinRTT(),
m_iFullBwCount(),
m_iCycleIndex(),
m_iLossEvents(),
m_iAckEvents(),
m_dLossEWMA(),
m_bFilledPipe(),
m_bProbeRTTDone()
{
}

CBBRv2CC::CBBRv2CC():
CBBRCC(BBR_V2)
{
}

CBBRv3CC::CBBRv3CC():
CBBRCC(BBR_V3)
{
}

void CBBRCC::init()
{
   m_BBRMode = BBR_STARTUP;
   m_LastUpdateTime = CTimer::getTime();
   m_LastRoundStart = m_LastUpdateTime;
   m_MinRTTStamp = m_LastUpdateTime;
   m_ProbeRTTDoneStamp = 0;
   m_dBtlBw = 0;
   m_iMinRTT = 0;
   m_iFullBwCount = 0;
   m_iCycleIndex = 0;
   m_iLossEvents = 0;
   m_iAckEvents = 0;
   m_dLossEWMA = 0.0;
   m_bFilledPipe = false;
   m_bProbeRTTDone = false;

   setACKTimer(m_iSYNInterval);
   m_dCWndSize = 16;
   m_dPktSndPeriod = 1;
}

double CBBRCC::getMinCWnd() const
{
   // Keep a smaller floor for narrowband/high-delay links to avoid persistent queueing.
   if ((m_iBandwidth > 0) && (m_iBandwidth <= 256))
      return 4.0;

   if ((m_dBtlBw > 0) && (m_iMinRTT > 0))
   {
      const double bdp = (m_dBtlBw * m_iMinRTT) / 1000000.0;
      if (bdp < 12.0)
         return 4.0;
      if (bdp < 20.0)
         return 8.0;
   }

   return 16.0;
}

void CBBRCC::enterMode(BBRMode mode)
{
   m_BBRMode = mode;
   if (BBR_PROBE_BW == mode)
      m_iCycleIndex = 0;
}

void CBBRCC::updateModel()
{
   if (m_iBandwidth > 0)
   {
      if (m_iBandwidth > m_dBtlBw)
      {
         m_dBtlBw = m_iBandwidth;
         if (BBR_STARTUP == m_BBRMode)
         {
            m_iFullBwCount = 0;
            m_bFilledPipe = false;
         }
      }
      else if (BBR_STARTUP == m_BBRMode)
      {
         ++ m_iFullBwCount;
         if (m_iFullBwCount >= 3)
            m_bFilledPipe = true;
      }
   }

   if ((m_iRTT > 0) && ((0 == m_iMinRTT) || (m_iRTT < m_iMinRTT)))
   {
      m_iMinRTT = m_iRTT;
      m_MinRTTStamp = CTimer::getTime();
   }
}

double CBBRCC::getStartupPacingGain(bool narrowband) const
{
   if (BBR_V2 == m_BBRVariant)
      return narrowband ? 1.35 : 1.7;
   if (BBR_V3 == m_BBRVariant)
      return narrowband ? 1.25 : 1.55;

   return narrowband ? 1.5 : 2.0;
}

double CBBRCC::getStartupCWndGain(bool narrowband) const
{
   if (BBR_V2 == m_BBRVariant)
      return narrowband ? 1.45 : 1.8;
   if (BBR_V3 == m_BBRVariant)
      return narrowband ? 1.35 : 1.65;

   return narrowband ? 1.6 : 2.0;
}

double CBBRCC::getDrainPacingGain(bool narrowband) const
{
   if (BBR_V2 == m_BBRVariant)
      return narrowband ? 0.9 : 0.82;
   if (BBR_V3 == m_BBRVariant)
      return narrowband ? 0.93 : 0.86;

   return narrowband ? 0.85 : 0.75;
}

double CBBRCC::getProbeRTTPacingGain() const
{
   if (BBR_V2 == m_BBRVariant)
      return 0.7;
   if (BBR_V3 == m_BBRVariant)
      return 0.75;

   return 0.6;
}

void CBBRCC::getProbeBwCycle(const double*& cycle, int& size) const
{
   static const double v1[] = {1.15, 0.85, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
   static const double v2[] = {1.08, 0.96, 1.03, 0.99, 1.0, 1.01, 0.99, 1.0};
   static const double v3[] = {1.05, 0.98, 1.02, 0.99, 1.0, 1.0, 0.99, 1.0};

   if (BBR_V2 == m_BBRVariant)
      cycle = v2;
   else if (BBR_V3 == m_BBRVariant)
      cycle = v3;
   else
      cycle = v1;

   size = 8;
}

double CBBRCC::getHighLossCWndCap() const
{
   if (BBR_V2 == m_BBRVariant)
      return 1.2;
   if (BBR_V3 == m_BBRVariant)
      return 1.1;

   return 1.3;
}

double CBBRCC::getLossPacingPenalty() const
{
   if (BBR_V2 == m_BBRVariant)
      return 1.12;
   if (BBR_V3 == m_BBRVariant)
      return 1.08;

   return 1.1;
}

double CBBRCC::getHighLossPacingPenalty() const
{
   if (BBR_V2 == m_BBRVariant)
      return 1.16;
   if (BBR_V3 == m_BBRVariant)
      return 1.12;

   return 1.2;
}

double CBBRCC::getLossCWndPenalty() const
{
   if (BBR_V2 == m_BBRVariant)
      return 0.88;
   if (BBR_V3 == m_BBRVariant)
      return 0.92;

   return 0.9;
}

double CBBRCC::getHighLossCWndPenalty() const
{
   if (BBR_V2 == m_BBRVariant)
      return 0.84;
   if (BBR_V3 == m_BBRVariant)
      return 0.88;

   return 0.8;
}

double CBBRCC::getTimeoutPacingPenalty() const
{
   if (BBR_V2 == m_BBRVariant)
      return 1.2;
   if (BBR_V3 == m_BBRVariant)
      return 1.15;

   return 1.25;
}

double CBBRCC::getTimeoutCWndPenalty() const
{
   if (BBR_V2 == m_BBRVariant)
      return 0.85;
   if (BBR_V3 == m_BBRVariant)
      return 0.9;

   return 0.8;
}

void CBBRCC::onACK(int32_t)
{
   uint64_t now = CTimer::getTime();
   if (now - m_LastUpdateTime < (uint64_t)m_iSYNInterval)
      return;

   m_LastUpdateTime = now;
   ++ m_iAckEvents;
   updateModel();

   if ((0 != m_iMinRTT) && (now - m_MinRTTStamp > 10000000ULL) && (BBR_PROBE_RTT != m_BBRMode))
   {
      enterMode(BBR_PROBE_RTT);
      m_bProbeRTTDone = false;
      m_ProbeRTTDoneStamp = 0;
   }

   if ((BBR_STARTUP == m_BBRMode) && m_bFilledPipe)
      enterMode(BBR_DRAIN);

   if (BBR_DRAIN == m_BBRMode)
   {
      if (m_dCWndSize <= 64)
         enterMode(BBR_PROBE_BW);
      else
         m_dCWndSize -= 8;
   }

   // EWMA loss sampling, used for narrowband links where random loss is significant.
   if (m_iAckEvents > 0)
   {
      const double sample = double(m_iLossEvents) / double(m_iAckEvents);
      m_dLossEWMA = m_dLossEWMA * 0.875 + sample * 0.125;
      m_iAckEvents = 0;
      m_iLossEvents = 0;
   }

   const bool narrowband = ((m_iBandwidth > 0) && (m_iBandwidth <= 256)) ||
                           ((m_dBtlBw > 0) && (m_dBtlBw < 384.0));
   const double min_cwnd = getMinCWnd();

   double pacing_gain = 1.0;
   double cwnd_gain = 2.0;

   if (BBR_STARTUP == m_BBRMode)
   {
      pacing_gain = getStartupPacingGain(narrowband);
      cwnd_gain = getStartupCWndGain(narrowband);
   }
   else if (BBR_DRAIN == m_BBRMode)
      pacing_gain = getDrainPacingGain(narrowband);
   else if (BBR_PROBE_BW == m_BBRMode)
   {
      const double* g_cycle = NULL;
      int cycle_size = 0;
      getProbeBwCycle(g_cycle, cycle_size);
      pacing_gain = g_cycle[m_iCycleIndex];
      if (now - m_LastRoundStart >= 1000000ULL)
      {
         m_iCycleIndex = (m_iCycleIndex + 1) % cycle_size;
         m_LastRoundStart = now;
      }
   }
   else if (BBR_PROBE_RTT == m_BBRMode)
   {
      pacing_gain = getProbeRTTPacingGain();
      cwnd_gain = 1.0;
      if (!m_bProbeRTTDone)
      {
         if (0 == m_ProbeRTTDoneStamp)
            m_ProbeRTTDoneStamp = now + 200000ULL;
         else if (now >= m_ProbeRTTDoneStamp)
            m_bProbeRTTDone = true;
      }
      else
      {
         m_MinRTTStamp = now;
         enterMode(BBR_PROBE_BW);
      }
   }

   // Persistent random loss usually means queue pressure on narrow paths; shrink probing amplitude.
   if (m_dLossEWMA > 0.05)
   {
      pacing_gain = 0.95 + (pacing_gain - 1.0) * 0.5;
      const double cap = getHighLossCWndCap();
      if (cwnd_gain > cap)
         cwnd_gain = cap;
   }

   if (m_dBtlBw > 0)
      m_dPktSndPeriod = 1000000.0 / (m_dBtlBw * pacing_gain);
   else if (m_iRcvRate > 0)
      m_dPktSndPeriod = 1000000.0 / (m_iRcvRate * pacing_gain);

   if (m_dPktSndPeriod < 1.0)
      m_dPktSndPeriod = 1.0;

   double bdp = 16.0;
   if ((m_dBtlBw > 0) && (m_iMinRTT > 0))
      bdp = (m_dBtlBw * m_iMinRTT) / 1000000.0;
   else if ((m_iBandwidth > 0) && (m_iRTT > 0))
      bdp = (m_iBandwidth * m_iRTT) / 1000000.0;

   m_dCWndSize = bdp * cwnd_gain;
   if (m_dCWndSize < min_cwnd)
      m_dCWndSize = min_cwnd;
   if (m_dCWndSize > m_dMaxCWndSize)
      m_dCWndSize = m_dMaxCWndSize;
}

void CBBRCC::onLoss(const int32_t*, int)
{
   ++ m_iLossEvents;

   const double min_cwnd = getMinCWnd();

   if (m_dPktSndPeriod < 1000000.0)
      m_dPktSndPeriod *= (m_dLossEWMA > 0.05) ? getHighLossPacingPenalty() : getLossPacingPenalty();

   if (m_dCWndSize > min_cwnd)
      m_dCWndSize *= (m_dLossEWMA > 0.05) ? getHighLossCWndPenalty() : getLossCWndPenalty();

   if (m_dCWndSize < min_cwnd)
      m_dCWndSize = min_cwnd;
}

void CBBRCC::onTimeout()
{
   const double min_cwnd = getMinCWnd();

   if (m_dPktSndPeriod < 1000000.0)
      m_dPktSndPeriod *= getTimeoutPacingPenalty();
   if (m_dCWndSize > min_cwnd)
      m_dCWndSize *= getTimeoutCWndPenalty();
   if (m_dCWndSize < min_cwnd)
      m_dCWndSize = min_cwnd;

   enterMode(BBR_PROBE_BW);
}

CCCVirtualFactory* createDefaultCCFactory()
{
   const char* cc_algo = getenv("UDT_CC_ALGO");
   if (NULL != cc_algo)
   {
      if ((0 == strcmp(cc_algo, "bbr")) || (0 == strcmp(cc_algo, "BBR")) ||
          (0 == strcmp(cc_algo, "bbr1")) || (0 == strcmp(cc_algo, "BBR1")))
         return new CCCFactory<CBBRCC>;

      if ((0 == strcmp(cc_algo, "bbr2")) || (0 == strcmp(cc_algo, "BBR2")) ||
          (0 == strcmp(cc_algo, "bbrv2")) || (0 == strcmp(cc_algo, "BBRV2")))
         return new CCCFactory<CBBRv2CC>;

      if ((0 == strcmp(cc_algo, "bbr3")) || (0 == strcmp(cc_algo, "BBR3")) ||
          (0 == strcmp(cc_algo, "bbrv3")) || (0 == strcmp(cc_algo, "BBRV3")))
         return new CCCFactory<CBBRv3CC>;
   }

   return new CCCFactory<CUDTCC>;
}
