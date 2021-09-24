//===---ublox/ublox_structure.hpp - UBX message structures ------*- C++-*-===//
// BSD 3-Clause License
//
// Copyright (C) 2019 Integrated Solutions for Systems, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//===----------------------------------------------------------------------===//
///
/// \file
/// \brief    This file contains structures and constants used for parsing
///           u-blox UBX messages.
/// \author   David Hodo <david.hodo@is4s.com>
/// \date     June 1, 2012
///
//===----------------------------------------------------------------------===//
#ifndef UBLOX__UBLOX_STRUCTURES_HPP
#define UBLOX__UBLOX_STRUCTURES_HPP

#include <cstdint>

namespace ublox
{
static const uint16_t MAX_UBX_SIZE =
  10000;  // Maximum size of a Ublox log buffer

// TODO: update these for F9
static const uint8_t MAXCHAN = 50;  // Maximum number of signal channels
static const uint8_t MAX_SAT =
  120;  // maximum number of prns - max prn is 32 plus prn 0 is 33
static const uint8_t MAX_SIG =
  120;  // maximum number of signals tracked - 120 for F9
static const uint8_t MAX_BLOCKS = 10;  // maximum number of rf blocks (MON-RF)

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER  // using MSVC
#define PACK(__Declaration__) \
  __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#else
#define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))
#endif

//! Header prepended to ubx binary messages
static const uint8_t HDR_CHKSM_LENGTH =
  8;  //(includes "sync1 sync2 classid msgid length checksum")
static const uint8_t UBX_SYNC_BYTE_1 = 0xB5;
static const uint8_t UBX_SYNC_BYTE_2 = 0x62;

static const uint8_t HDR_LENGTH = 6;

//! UBX Protocol Class ID's
enum ublox_class_ids : uint8_t
{
  MSG_CLASS_ACK = 0X05,
  MSG_CLASS_AID = 0x0B,
  MSG_CLASS_CFG = 0x06,
  MSG_CLASS_ESF = 0x10,
  MSG_CLASS_HNR = 0x28,
  MSG_CLASS_INF = 0x04,
  MSG_CLASS_LOG = 0x21,
  MSG_CLASS_MGA = 0x13,
  MSG_CLASS_MON = 0x0A,
  MSG_CLASS_NAV = 0x01,
  MSG_CLASS_RXM = 0x02,
  MSG_CLASS_SEC = 0x27,
  MSG_CLASS_TIM = 0x0D,
  MSG_CLASS_UPD = 0x09
};

//! UBX Class ACK Msg IDs
enum ublox_class_ack_msg_ids : uint8_t
{
  MSG_ID_ACK_ACK = 0x01,
  MSG_ID_ACK_NAK = 0x00
};

//! UBX Class AID Msg IDs
enum ublox_class_aid_msg_ids : uint8_t
{
  MSG_ID_AID_ALM    = 0x30,
  MSG_ID_AID_ALPSRV = 0X32,
  MSG_ID_AID_ALP    = 0x50,
  MSG_ID_AID_AOP    = 0x33,
  MSG_ID_AID_DATA   = 0x10,
  MSG_ID_AID_EPH    = 0x31,
  MSG_ID_AID_HUI    = 0x02,
  MSG_ID_AID_INI    = 0x01,
  MSG_ID_AID_REQ    = 0x00
};

//! UBX Class CFG Msg IDs
enum ublox_class_cfg_msg_ids : uint8_t
{
  MSG_ID_CFG_ANT       = 0X13,
  MSG_ID_CFG_CNFGR     = 0x09,
  MSG_ID_CFG_DAT       = 0x06,
  MSG_ID_CFG_DGNSS     = 0x70,
  MSG_ID_CFG_DOSC      = 0x61,
  MSG_ID_CFG_DYNSEED   = 0x85,
  MSG_ID_CFG_ESRC      = 0x60,
  MSG_ID_CFG_FIXSEED   = 0x84,
  MSG_ID_CFG_GEOFENCE  = 0x69,
  MSG_ID_CFG_GNSS      = 0x3E,
  MSG_ID_CFG_HNR       = 0x5C,
  MSG_ID_CFG_INF       = 0x02,
  MSG_ID_CFG_ITFM      = 0x39,
  MSG_ID_CFG_LOGFILTER = 0x47,
  MSG_ID_CFG_MSG       = 0x01,
  MSG_ID_CFG_EKF       = 0x12,
  MSG_ID_CFG_ESFGWT    = 0x29,
  MSG_ID_CFG_FXN       = 0x0E,
  MSG_ID_CFG_NAV5      = 0x24,
  MSG_ID_CFG_NAVX5     = 0x23,
  MSG_ID_CFG_NMEA      = 0x17,
  MSG_ID_CFG_ODO       = 0x1E,
  MSG_ID_CFG_NVS       = 0x22,
  MSG_ID_CFG_PM        = 0x32,
  MSG_ID_CFG_PM2       = 0x3B,
  MSG_ID_CFG_PMS       = 0x86,
  MSG_ID_CFG_PRT       = 0x00,
  MSG_ID_CFG_PWR       = 0x57,
  MSG_ID_CFG_RATE      = 0x08,
  MSG_ID_CFG_RINV      = 0x34,
  MSG_ID_CFG_RST       = 0x04,
  MSG_ID_CFG_RXM       = 0x11,
  MSG_ID_CFG_SBAS      = 0x16,
  MSG_ID_CFG_SMGR      = 0x62,
  MSG_ID_CFG_TMODE     = 0x1D,
  MSG_ID_CFG_TMODE2    = 0x3D,
  MSG_ID_CFG_TMODE3    = 0x71,
  MSG_ID_CFG_TP        = 0x07,
  MSG_ID_CFG_TP5       = 0x31,
  MSG_ID_CFG_TXSLOT    = 0x53,
  MSG_ID_CFG_USB       = 0x1B,
  MSG_ID_CFG_VALDEL    = 0x8C,
  MSG_ID_CFG_VALGET    = 0x8B,
  MSG_ID_CFG_VALSET    = 0x8A
};

//! UBX Class ESF Msg IDs
enum ublox_class_esf_msg_ids : uint8_t
{
  MSG_ID_ESF_INS    = 0x15,
  MSG_ID_ESF_MEAS   = 0x02,
  MSG_ID_ESF_RAW    = 0x03,
  MSG_ID_ESF_STATUS = 0x10
};

//! UBX Class HNR Msg IDs
enum ublox_class_hnr_msg_ids : uint8_t
{
  MSG_HNR_PVT = 0x00
};

//! UBX Class INF Msg IDs
enum ublox_class_inf_msg_ids : uint8_t
{
  MSG_ID_INF_DEBUG   = 0x04,
  MSG_ID_INF_ERROR   = 0x00,
  MSG_ID_INF_NOTICE  = 0x02,
  MSG_ID_INF_TEST    = 0x03,
  MSG_ID_INF_WARNING = 0x01
};

//! UBX Class LOG Msg IDs
enum ublox_class_log_msg_ids : uint8_t
{
  MSG_ID_LOG_CREATE           = 0X07,
  MSG_ID_LOG_ERASE            = 0X03,
  MSG_ID_LOG_FINDTIME         = 0X0E,
  MSG_ID_LOG_INFO             = 0X08,
  MSG_ID_LOG_RETRIEVEPOSEXTRA = 0x0F,
  MSG_ID_LOG_RETRIEVEPOS      = 0X0B,
  MSG_ID_LOG_RETRIEVESTRING   = 0X0D,
  MSG_ID_LOG_RETRIEVE         = 0X09,
  MSG_ID_LOG_STRING           = 0X04
};

//! UBX Class MGA Msg IDs
enum ublox_class_mga_msg_ids : uint8_t
{
  MSG_ID_MGA_ACK   = 0x60,
  MSG_ID_MGA_ANO   = 0x20,
  MSG_ID_MGA_BDS   = 0x03,
  MSG_ID_MGA_DBD   = 0x80,
  MSG_ID_MGA_FLASH = 0x21,
  MSG_ID_MGA_GAL   = 0x02,
  MSG_ID_MGA_GLO   = 0x06,
  MSG_ID_MGA_GPS   = 0x00,
  MSG_ID_MGA_INI   = 0x40,
  MSG_ID_MGA_QZSS  = 0x05
};

//! UBX Class MON Msg IDs
enum ublox_class_mon_msg_ids : uint8_t
{
  MSG_ID_MON_GNSS  = 0x28,
  MSG_ID_MON_HW2   = 0x0B,
  MSG_ID_MON_HW    = 0x09,
  MSG_ID_MON_IO    = 0x02,
  MSG_ID_MON_MSGPP = 0x06,
  MSG_ID_MON_PATCH = 0x27,
  MSG_ID_MON_RXBUF = 0x07,
  MSG_ID_MON_RXR   = 0x21,
  MSG_ID_MON_SMGR  = 0x2E,
  MSG_ID_MON_SPAN  = 0x31,
  MSG_ID_MON_TXBUF = 0X08,
  MSG_ID_MON_VER   = 0x04,
  MSG_ID_MON_RF    = 0x38
};

//! UBX Class NAV Msg IDs
enum ublox_class_nav_msg_ids : uint8_t
{
  MSG_ID_NAV_AOPSTATUS = 0x60,
  MSG_ID_NAV_ATT       = 0x05,
  MSG_ID_NAV_CLOCK     = 0x22,
  MSG_ID_NAV_DGPS      = 0x31,
  MSG_ID_NAV_DOP       = 0x04,
  MSG_ID_NAV_EKFSTATUS = 0x40,
  MSG_ID_NAV_EOE       = 0x61,
  MSG_ID_NAV_GEOFENCE  = 0x39,
  MSG_ID_NAV_HPPOSECEF = 0x13,
  MSG_ID_NAV_HPPOSLLH  = 0x14,
  MSG_ID_NAV_ODO       = 0x09,
  MSG_ID_NAV_ORB       = 0x34,
  MSG_ID_NAV_POSECEF   = 0x01,
  MSG_ID_NAV_POSLLH    = 0x02,
  MSG_ID_NAV_PVT       = 0x07,
  MSG_ID_NAV_RELPOSNED = 0x3C,
  MSG_ID_NAV_RESETODO  = 0x10,
  MSG_ID_NAV_SAT       = 0x35,
  MSG_ID_NAV_SIG       = 0x43,
  MSG_ID_NAV_SBAS      = 0x32,
  MSG_ID_NAV_SOL       = 0x06,
  MSG_ID_NAV_STATUS    = 0x03,
  MSG_ID_NAV_SVINFO    = 0x30,
  MSG_ID_NAV_SVIN      = 0x3B,
  MSG_ID_NAV_TIMEBDS   = 0x24,
  MSG_ID_NAV_TIMEGAL   = 0x25,
  MSG_ID_NAV_TIMEGLO   = 0x23,
  MSG_ID_NAV_TIMEGPS   = 0x20,
  MSG_ID_NAV_TIMELS    = 0x26,
  MSG_ID_NAV_TIMEUTC   = 0x21,
  MSG_ID_NAV_VELECEF   = 0x11,
  MSG_ID_NAV_VELNED    = 0x12
};

//! UBX Class RXM Msg IDs
enum ublox_class_rxm_msg_ids : uint8_t
{
  MSG_ID_RXM_ALM   = 0x30,
  MSG_ID_RXM_EPH   = 0x31,
  MSG_ID_RXM_IMES  = 0x61,
  MSG_ID_RXM_MEASX = 0x14,
  MSG_ID_RXM_PMREQ = 0x41,
  MSG_ID_RXM_RAW   = 0x10,
  MSG_ID_RXM_RAWX  = 0x15,
  MSG_ID_RXM_RLM   = 0x59,
  MSG_ID_RXM_RTCM  = 0x32,
  MSG_ID_RXM_SFRB  = 0x11,
  MSG_ID_RXM_SFRBX = 0x13,
  MSG_ID_RXM_SVSI  = 0x20
};

//! UBX Class SEC Msg IDs
enum ublox_class_sec_msg_ids : uint8_t
{
  MSG_ID_SEC_SIGN   = 0x01,
  MSG_ID_SEC_UNIQID = 0x03
};

//! UBX Class TIM Msg IDs
enum ublox_class_tim_msg_ids : uint8_t
{
  MSG_ID_TIM_DOSC   = 0x11,
  MSG_ID_TIM_FCHG   = 0x16,
  MSG_ID_TIM_HOC    = 0x17,
  MSG_ID_TIM_SMEAS  = 0x13,
  MSG_ID_TIM_SVIN   = 0x04,
  MSG_ID_TIM_TM2    = 0x03,
  MSG_ID_TIM_TOS    = 0x12,
  MSG_ID_TIM_TP     = 0x01,
  MSG_ID_TIM_VCOCAL = 0x15,
  MSG_ID_TIM_VRFY   = 0x06
};

//! UBX Class UPD Msg IDs
enum ublox_class_upd_msg_ids : uint8_t
{
  MSG_ID_UPD_SOS = 0x14
};

static const float LAT_LONG_SCALING = 1e-7;
static const float MM_TO_M          = 1e-3;

enum class ResetMask
{
  HotStart  = 0x0000,
  WarmStart = 0x0001,
  ColdStart = 0xFFFF
};

enum ResetMode
{
  Hardware                   = 0x00,
  ControlledSoftware         = 0x01,
  ControlledSoftwareGps      = 0x02,
  HardwareResetAfterShutdown = 0x04,
  ControlledGpsStop          = 0x08,
  ControllerGpsStart         = 0x09
};

//! Ublox protocol GNSS identifier
enum class GnssId : uint8_t
{
  GPS     = 0,
  Galileo = 2,
  BeiDou  = 3,
  QZSS    = 5,
  GLONASS = 6,
};

//! Ublox Quality Indicator
enum class QualityIndicator : uint8_t
{
  NoSignal              = 0,  //!< no signal
  Searching             = 1,  //!< searching signal
  Acquired              = 2,  //!< signal acquired
  Unusable              = 3,  //!< signal detected but unusable
  CodeLocked            = 4,  //!< code locked and time synchronized
  CodeAndCarrierLocked  = 5,  //!< Code and carrier locked
  CodeAndCarrierLocked2 = 6,  //!< Code and carrier locked
  CodeAndCarrierLocked3 = 7,  //!< Code and carrier locked
};

//! Ublox Correction Source Indicator
enum class CorrectionSource : uint8_t
{
  None      = 0,
  SBAS      = 1,
  BeiDou    = 2,
  RTCM2     = 3,
  RTCM3_OSR = 4,
  RTCM3_SSR = 5,
  QZSS_SLAS = 6
};

//! Ublox Ionospheric Model Used
enum class IonoModel : uint8_t
{
  NoModel         = 0,  //!< No Model
  KlobucharGps    = 1,  //!< Klobuchar model transmitted by GPS
  SBAS            = 2,  //!< SBAS model
  KlobucharBeiDou = 3,  //!< Klobuchar model transmitted by BeiDou
  DualFrequency   = 8   //!< Iono delay derived from dual frequency observations
};

enum Message_ID
{
  //! UBXClass ACK
  ACK_ACK = 1281,  //!< (ID 0x05 0x01) Message Acknowledged
  ACK_NAK = 1280,  //!< (ID 0x05 0x00) Message Not-Acknowledged
                   //! UBXClass AID
  AID_ALM = 2864,  //!< (ID 0x0B 0x30) Almanac
  AID_AOP = 2865,  //!< (ID 0x0B 0x33) AssistNow Autonomous data
  AID_EPH = 2865,  //!< (ID 0x0B 0x31) Ephemerides
  AID_HUI = 2818,  //!< (ID 0x0B 0x02) GPS Health, Ionospheric, UTC
  AID_INI = 2817,  //!< (ID 0x0B 0x01) Position, Time, Frequency, Clock Drift
  AID_REQ = 2816,  //!< (ID 0x0B 0x00) Receiver Requests Aiding data if not
                   //!< present at startup
                   //! UBXClass CFG
  CFG_ANT     = 1555,  //!< (ID 0x06 0x13) Antenna Control Settings
  CFG_CNFGR   = 1545,  //!< (ID 0x06 0x09) Clear, Save and Load configurations
  CFG_DAT     = 1542,  //!< (ID 0x06 0x06) Set User-defined Datum.
  CFG_DGNSS   = 1648,  //!< (ID 0x06 0x70) DGNSS Configuration
  CFG_DOSC    = 1633,  //!< (ID 0x06 0x61) Disciplined oscillator configuration
  CFG_DYNSEED = 1669,  //!< (ID 0x06 0x85) Programming the dynamic seed for the
                       //!< host interface signature
  CFG_ESRC =
    1632,  //!< (ID 0x06 0x60) External synchronization source configuration
  CFG_FIXSEED = 1668,   //!< (ID 0x06 0x84) Programming the fixed seed for host
                        //!< interface signature
  CFG_GEOFENCE = 1641,  //!< (ID 0x06 0x69) Geofencing configuration
  CFG_GNSS     = 1598,  //!< (ID 0x06 0x3E) GNSS System Configuratoin Setttings
  CFG_HNR      = 1628,  //!< (ID 0x06 0x5C) High Navigation Rate Settings
  CFG_INF      = 1528,  //!< (ID 0x06 0x02) Poll configuration for one protocol
  CFG_ITFM =
    1593,  //!< (ID 0x06 0x39) Jamming/Interference Monitor configuration
  CFG_LOGFILTER = 1607,  //!< (ID 0x06 0x47) Data Logger Configuration
  CFG_MSG       = 1537,  //!< (ID 0x06 0x01) Message configuration
  CFG_NAV5  = 1572,  //!< (ID 0x06 0x24) Navigation Algorithm Parameter Settings
  CFG_NAVX5 = 1571,  //!< (ID 0x06 0x23) Navigation Engine Expert Settings
  CFG_NMEA  = 1559,  //!< (ID 0x06 0x17) NMEA protocol configuration
  CFG_ODO   = 1566,  //!< (ID 0x06 0x1E) Odometer, Low-speed COG Engine Settings
  CFG_PM2   = 1595,  //!< (ID 0x06 0x3B) Extended Power Management configuration
  CFG_PMS   = 1670,  //!< (ID 0x06 0x86) Power Mode Setup
  CFG_PRT   = 1536,  //!< (ID 0x06 0x00) I/O Protocol Settings
  CFG_PWR   = 1623,  //!< (ID 0x06 0x57) Put receiver in a defined power state.
  CFG_RATE  = 1544,  //!< (ID 0x06 0x08) Navigation/Measurement Rate Settings
  CFG_RINV  = 1588,  //!< (ID 0x06 0x34) Contents of Remote Inventory
  CFG_RST =
    1540,  //!< (ID 0x06 0x04) Reset Receiver / Clear Backup Data Structures
  CFG_RXM    = 1553,  //!< (ID 0x06 0x11) RXM configuration
  CFG_SBAS   = 1558,  //!< (ID 0x06 0x16) SBAS Configuration
  CFG_SMGR   = 1634,  //!< (ID 0x06 0x62) Synchronization manager configuration
  CFG_TMODE2 = 1597,  //!< (ID 0x06 0x3D) Time Mode Settings 2
  CFG_TMODE3 = 1649,  //!< (ID 0x06 0x71) Time Mode Settings 3
  CFG_TP5 =
    1585,  //!< (ID 0x06 0x31) Poll Time Pulse Parameters for Time Pulse 0
  CFG_TXSLOT = 1619,  //!< (ID 0x06 0x53) TX buffer time slots configuration
  CFG_USB    = 1563,  //!< (ID 0x06 0x1B) USB Configuration
                   //! UBXClass ESF
  ESF_INS  = 1045,  //!< (ID 0x10 0x15) Vehicle dynamics information
  ESF_MEAS = 1026,  //!< (ID 0x10 0x02) External Sensor Fusion Measurements
  ESF_RAW  = 1027,  //!< (ID 0x10 0x03) Raw sensor measurements
  ESF_STATUS =
    1040,  //!< (ID 0x10 0x10) External Sensor Fusion (ESF) status information
           //! UBXClass HNR
  HNR_PVT = 10240,  //!< (ID 0x28 0x00) High Rate Output of PVT Solution
                    //! UBXClass INF
  INF_DEBUG = 1028,  //!< (ID 0x04 0x04) ASCII output with debug contents
  INF_ERROR = 1024,  //!< (ID 0x04 0x00) ASCII output with error contents
  INF_NOTICE =
    1026,  //!< (ID 0x04 0x02) ASCII output with informational contents
  INF_TEST    = 1027,  //!< (ID 0x04 0x03) ASCII output with test contents
  INF_WARNING = 1025,  //!< (ID 0x04 0x01) ASCII output with warning contents
                       //! UBXClass LOG
  LOG_CREATE = 8455,  //!< (ID 0x21 0x07) Create Log File
  LOG_ERASE  = 8451,  //!< (ID 0x21 0x03) Erase Logged Data
  LOG_FINDTIME =
    8462,  //!< (ID 0x21 0x0E) Find index of a log entry based on a given time
  LOG_INFO             = 8456,  //!< (ID 0x21 0x08) Log information
  LOG_RETRIEVEPOSEXTRA = 8463,  //!< (ID 0X21 0x0F) Odometer log entry
  LOG_RETRIEVEPOS      = 8459,  //!< (ID 0x21 0x0B) Position fix log entry
  LOG_RETRIEVESTRING   = 8461,  //!< (ID 0x21 0x0D) Byte string log entry
  LOG_RETRIEVE         = 8457,  //!< (ID 0x21 0x09) Request log data
  LOG_STRING =
    8452,  //!< (ID 0x21 0x09) Store arbitrary string in on-board flash
           //! UBXClass MGA
  MGA_ACK = 4960,  //!< (ID 0x13 0x60) Multiple GNSS Acknowledge message
  MGA_ANO =
    4896,  //!< (ID 0x13 0x20) Multiple GNSS AssistNow Offline Assistance
  MGA_BDS   = 4867,  //!< (ID 0x13 0x03) BDS Ephemeris Assistance
  MGA_DBD   = 4992,  //!< (ID 0x13 0x80) Navigation Database
  MGA_FLASH = 4897,  //!< (ID 0x13 0x21) Transfer MGA-ANO data block to flash
  MGA_GAL   = 4866,  //!< (ID 0x13 0x02) Galileo Ephemeris Assistance
  MGA_GLO   = 4870,  //!< (ID 0x13 0x06) GLONASS Ephemeris Assistance
  MGA_GPS   = 4864,  //!< (ID 0x13 0x00) GPS Ephemeris Assistance
  MGA_INI   = 4928,  //!< (ID 0x13 0x40) Initial Position Assistance
  MGA_QZSS  = 4869,  //!< (ID 0x13 0x05) QZSS Ephemeris Assistance
                    //! UBXClass MON
  MON_GNSS = 2600,  //!< (ID 0x0A 0x28) Information message major GNSS selection
  MON_HW2  = 2571,  //!< (ID 0x0A 0x0B) Extended Hardware Status
  MON_HW   = 2569,  //!< (ID 0x0A 0x09) Hardware Status
  MON_IO   = 2562,  //!< (ID 0x0A 0x02) I/O Subsystem Status
  MON_MSGPP = 2566,  //!< (ID 0x0A 0x06) Message Parse and Process Status
  MON_PATCH = 2599,  //!< (ID 0x0A 0x27) Request for installed patches
  MON_RXBUF = 2567,  //!< (ID 0x0A 0x07) Receiver Buffer Status
  MON_RXR   = 2593,  //!< (ID 0x0A 0x21) Receiver Status Information
  MON_SMGR  = 2606,  //!< (ID 0x0A 0x2E) Synchronization Manager Status
  MON_TXBUF = 2568,  //!< (ID 0x0A 0x08) Transmitter Buffer Status
  MON_VER   = 2564,  //!< (ID 0x0A 0x04) Reciever/Software/ROM Version
  MON_SPAN  = 2609,  //!< (ID 0x0A 0x31) Spectrum monitoring
  MON_RF    = 2616,  //!< (ID 0x0A 0x38) RF status
                  //! UBXClass NAV
  NAV_AOPSTATUS = 352,  //!< (ID 0x01 0x60) AssistNow Autonomous Status
  NAV_ATT       = 261,  //!< (ID 0x01 0x05) Attitude Solution
  NAV_CLK       = 290,  //!< (ID 0x01 0x22) Clock information
  NAV_DGPS =
    305,  //!< (ID 0x01 0x31) Outputs correction data used for the nav solution
  NAV_DOP      = 260,  //!< (ID 0x01 0x04) Various Dilution of Precisions
  NAV_EOE      = 353,  //!< (ID 0x01 0x61) End Of Epoch
  NAV_GEOFENCE = 313,  //!< (ID 0x01 0x39) Geofencing status
  NAV_HPPOSECEF =
    275,  //!< (ID 0x01 0x13) High Precision Position Solution in ECEF
  NAV_HPPOSLLH =
    276,  //!< (ID 0x01 0x14) High Precision Geodetic Position Solution
  NAV_ODO     = 265,  //!< (ID 0x01 0x09) Odometer Solution
  NAV_ORB     = 308,  //!< (ID 0x01 0x34) GNSS Orbit Database Info
  NAV_POSECEF = 257,  //!< (ID 0x01 0x01) ECEF Position
  NAV_POSLLH  = 258,  //!< (ID 0x01 0x02) Pos (Lat,Long,Height)
  NAV_PVT = 263,  //!< (ID 0x01 0x07) Navigation Position Velocity Time Solution
  NAV_RELPOSNED =
    316,  //!< (ID 0x01 0x3C) Relative Positioning Information in NED frame
  NAV_RESETODO = 272,  //!< (ID 0x01 0x10) Reset odometer
  NAV_SAT      = 309,  //!< (ID 0x01 0x35) Satellite Information
  NAV_SIG      = 323,  // (ID 0x01 0x43) Signal Information
  NAV_SBAS     = 306,  //!< (ID 0x01 0x32) SBAS Status Data
  NAV_SOL      = 262,  //!< (ID 0x01 0x06) ECEF Pos,Vel, TOW, Accuracy,
  NAV_STATUS =
    259,  //!< (ID 0x01 0x03) TTFF, GPS Fix type, time since startup/reset
  NAV_SVINFO =
    304,  //!< (ID 0x01 0x30) Info on Channels and the SVs they're tracking
  NAV_SVIN    = 315,  //!< (ID 0x01 0x3B) Survey-in data
  NAV_TIMEBDS = 292,  //!< (ID 0x01 0x24) BDS Time Solution
  NAV_TIMEGAL = 293,  //!< (ID 0x01 0x25) Galileo Time Solution
  NAV_TIMEGLO = 291,  //!< (ID 0x01 0x23) GLO Time Solution
  NAV_TIMEGPS = 288,  //!< (ID 0x01 0x20) GPS Time
  NAV_LSTIME  = 294,  //!< (ID 0x01 0x26) Leap second event information
  NAV_TIMEUTC = 289,  //!< (ID 0x01 0x21) UTC Time
  NAV_VELECEF = 273,  //!< (ID 0x01 0x11) Velocity Solution in ECEF
  NAV_VELNED =
    274,  //!< (ID 0x01 0x12) Vel (North, East, Down), Speed, Ground Speed
          //! UBXClass RXM
  RXM_ALM   = 560,  //!< (ID 0x02 0x30) GPS Constellation Almanac Data for a SV
  RXM_EPH   = 561,  //!< (ID 0x02 0x31) GPS Constellation Ephemeris Data
  RXM_IMES  = 609,  //!< (ID 0x02 0x61) Indoor Messaging System Information
  RXM_MEASX = 532,  //!< (ID 0x02 0x14) Satellite Measurements for RRLP
  RXM_PMREQ = 577,  //!< (ID 0x02 0x41) GPS Constellation Ephemeris Data
  RXM_RAW   = 528,  //!< (ID 0x02 0x10) Raw Measurement Data
  RXM_RAWX  = 533,  //!< (ID 0x02 0x15) Raw DGPS Data
  RXM_RLM   = 601,  //!< (ID 0x02 0x59) Galileo SAR (Long/Short)-RLM report
  RXM_RTCM  = 562,  //!< (ID 0x02 0x32) RTCM Input Status
  RXM_SFRB  = 529,  //!< (ID 0x02 0x11) Subframe Buffer
  RXM_SFRBX = 531,  //!< (ID 0x02 0x13) Broadcast Navigation Data Subframe
  RXM_SVSI  = 544,  //!< (ID 0x02 0x20) SV Status Info
                    //! UBXClass SEC
  SEC_SIGN   = 9985,  //!< (ID 0x27 0x01) Signature of a previous message
  SEC_UNIQID = 9987,  //!< (ID 0x27 0x03) Unique Chip ID
                      //! UBXClass TIM
  TIM_DOSC = 3345,    //!< (ID 0x0D 0x11) Disciplined oscillator control
  TIM_FCHG =
    3350,  //!< (ID 0x0D 0x16) Oscillator frequency changed notification
  TIM_HOC    = 3351,  //!< (ID 0x0D 0x17) Host oscillator control
  TIM_SMEAS  = 3347,  //!< (ID 0x0D 0x13) Source measurement
  TIM_SVIN   = 3332,  //!< (ID 0x0D 0x04) Survey-in data
  TIM_TM2    = 3331,  //!< (ID 0x0D 0x03) Time mark data
  TIM_TOS    = 3346,  //!< (ID 0x0D 0x12) Time Pulse Time and Frequency Data
  TIM_TP     = 3329,  //!< (ID 0x0D 0x01) Time Pulse Timedata
  TIM_VCOCAL = 3349,  //!< (ID 0x0D 0x15) Stop calibration/VCO calibration
                      //!< extended command/Results of the calibration
  TIM_VRFY = 3334,    //!< (ID 0x0D 0x06) Sourced Time Verification
                      //! UBXClass UPD
  UPD_SOS = 2324,     //!< (ID 0x09 0x14) Backup File Restore Status
};

PACK(struct UbloxHeader {
  uint8_t sync1;          //!< start of packet first byte (0xB5)
  uint8_t sync2;          //!< start of packet second byte (0x62)
  uint8_t message_class;  //!< Class that defines basic subset of message (NAV,
                          // RXM, etc.)
  uint8_t  message_id;    //!< Message ID
  uint16_t payload_length;  //!< length of the payload data, excluding header
                            //!< and checksum
});

///////////////////////////////////////////////////////////
// Configuration Messages
///////////////////////////////////////////////////////////
/*!
 * CFM-MSG Message Structure
 * This message requests a specifiable message at a given rate.
 * ID: 0x06  0x01 Payload Length=3 bytes
 */
PACK(struct CfgMsgRate {
  UbloxHeader header;         //!< Ublox header
  uint8_t     message_class;  //!< class of message to request
  uint8_t     message_id;     //!< id of message to request
  uint8_t     rate;           //!< rate message will be sent
  uint8_t     checksum[2];
});

/*!
 * CFM-MSG Message Structure
 * This message requests a message once.
 * ID: 0x06  0x01 Payload Length=2 bytes
 */
PACK(struct CfgMsg {
  UbloxHeader header;         //!< Ublox header
  uint8_t     message_class;  //!< class of message to request
  uint8_t     message_id;     //!< id of message to request
  uint8_t     checksum[2];
});

/*!
 * CFG-CFG Message Structure
 * This message clears, saves, or loads novalitle memory.
 * Set masks to 0x061F to clear, save, or load all values.
 * ID: 0x06  0x09 Payload Length=12 bytes
 */
PACK(struct CfgCfg {
  UbloxHeader header;       //!< Ublox header
  uint32_t    clear_mask;   //!< clear mask
  uint32_t    save_mask;    //!< save mask
  uint32_t    load_mask;    //!< load mask
  uint8_t     checksum[2];  //!< Checksum
});

/*!
 * CFM-RST Message Structure
 * This message allows a receiver to be reset.
 * ID: 0x06  0x04 Payload Length=4 bytes
 */
PACK(struct CfgRst {
  UbloxHeader header;     //!< Ublox header
  uint16_t nav_bbr_mask;  //!< Nav data to clear: 0x0000 = hot start, 0x0001 =
                          // warm start, 0xFFFF=cold start
  uint8_t reset_mode;     //!< Reset mode
  uint8_t reserved;       //!< reserved
  uint8_t checksum[2];
});

/*!
 * CFM-PRT Message Structure
 * This message configures a USART or USB port.
 * Use to specify input/output protocols to use
 * ID: 0x06  0x00 Payload Length=20 bytes
 */
PACK(struct CfgPrt {
  UbloxHeader header;  //!< Ublox header
  uint8_t  port_id;    //!< port identifier (1 or 2-UART or 3-USB, 4-SPI, 0-DDC)
  uint8_t  reserved;   //!< reserved
  uint16_t tx_ready;   //!< transmit ready status
  uint32_t reserved2;  //!< UART mode
  uint32_t reserved3;  //!< baudrate
  uint16_t input_mask;   //!< input protocol mask
  uint16_t output_mask;  //!< output protocol mask
  uint16_t reserved4;    //!< reserved (always 0)
  uint16_t reserved5;    //!< reserved (always 0)
  uint8_t  checksum[2];
});

// Values for CfgPrt.port_id
enum PortIdentifier
{
  PORT_ID_DDC   = 0,
  PORT_ID_UART1 = 1,
  PORT_ID_UART2 = 2,
  PORT_ID_USB   = 3,
  PORT_ID_SPO   = 4
};

/*!
 * CFG-NAV5 Message Structure
 * This message configures Navigation algorithm
 * parameters.
 * ID: 0x06  0x24 Payload Length=36 bytes
 */
PACK(struct CfgNav5 {
  UbloxHeader header;  //!< Ublox header
  uint16_t    mask;    //!< parameters bitmask (only masked params applied)
  uint8_t     dynamic_model;            //!< dynamic platform
  uint8_t     fix_mode;                 //!< positioning fix mode
  int32_t     fixed_altitude;           //!< (scale .01) (m)
  uint32_t    fixed_altitude_variance;  //!< (scale .0001) (m^2)
  int8_t      min_elevation;            //!< (deg)
  uint8_t     dead_reckoning_limit;  //!< max time to perform DR w/out GPS (sec)
  uint16_t    pdop;                  //!< (scale .1)
  uint16_t    tdop;                  //!< (scale .1)
  uint16_t    pos_accuracy_mask;     //!< (m)
  uint16_t    time_accuracy_mask;    //!< (m)
  uint8_t     static_hold_threshold;  //!<
  uint8_t     dgps_timeout;           //!<
  uint32_t    reserved2;              //!< reserved (always set to zero)
  uint32_t    reserved3;              //!< reserved (always set to zero)
  uint32_t    reserved4;              //!< reserved (always set to zero)
  uint8_t     checksum[2];
});

/////////////////////////////////////////////////////////////
// Navigation Messages
/////////////////////////////////////////////////////////////
/*!
 * NAV_HPPOSECEF Message Structure
 * This message outputs an high precision Geodetic position in ECEF coordinates
 * ID: 0x01  0x013  Payload Length=28 bytes
 */
PACK(struct NavHPPosECEF {
  UbloxHeader header;
  uint8_t     version;
  uint8_t     reserved1[3];
  uint32_t    iTOW;  //!< GPS millisecond time of week
  int32_t     x;     //!< ECEF X coordinate [cm]
  int32_t     y;     //!< ECEF Y coordinate [cm]
  int32_t     z;     //!< ECEF Z coordinate [cm]
  int8_t      xHp;   //!< ECEF X high precision component [mm]
  int8_t      yHp;   //!< ECEF Y high precision component [mm]
  int8_t      zHp;   //!< ECEF Z high precision component [mm]
  int8_t      reserved2;
  uint32_t    posAcc;  //!< Position accuracy estimate [cm]
  uint8_t     checksum[2];
});

/*!
 * NAV_HPPOSLLH Message Structure
 * This message outputs an high precision Geodetic position in ECEF coordinates
 * ID: 0x01  0x02  Payload Length=28 bytes
 */

PACK(struct NavHPPosLLH {
  UbloxHeader header;  //!< Ublox header
  uint8_t     version;
  uint8_t     reserved1[3];
  uint32_t    iTOW;      //!< GPS millisecond time of week
  int32_t     lon;       //!< Longitude[deg]
  int32_t     lat;       //!< Latitude[deg]
  int32_t     height;    //!< Height above ellipsoid.[mm]
  int32_t     hMSL;      //!< Height above mean sea level [cm]
  int8_t      latHp;     //!< [deg]
  int8_t      lonHp;     //!< [deg]
  int8_t      heightHp;  //!< [mm]
  int8_t      hMSLHp;    //!< [mm]
  int32_t     hAcc;      //!< [mm]
  int32_t     vAcc;      //!< [mm]
  uint8_t     checksum[2];
});
/*!
 * NAV-STATUS Message Structure
 * This message contains gps fix type and ttff
 * ID: 0x01 0x03 Payload Length=16 bytes
 */
static const uint8_t NAVSTATUS_FLAG_GPSFIX_VALID      = 1 << 0;
static const uint8_t NAVSTATUS_FLAG_DGPS_USED_FOR_FIX = 1 << 1;
static const uint8_t NAVSTATUS_FLAG_WEEK_NUM_VALID    = 1 << 2;
static const uint8_t NAVSTATUS_FLAG_TOW_VALID         = 1 << 3;

// Flags2 Spoofing Dection State
static const uint8_t NAVSTATUS_FLAGS2_SPOOFDETSTATE_UNKNOWN = 0x00;
static const uint8_t NAVSTATUS_FLAGS2_SPOOFDETSTATE_NOSPOOFING = 0x08;
static const uint8_t NAVSTATUS_FLAGS2_SPOOFDETSTATE_SPOOFING = 0x10;
static const uint8_t NAVSTATUS_FLAGS2_SPOOFDETSTATE_MULTIPLESPOOFING = 0x18;

PACK(struct NavStatus {
  UbloxHeader header;
  uint32_t    iTOW;  // Time of Week (ms)
  uint8_t
    fixtype;      // no fix=0x00, deadreckoning only=0x01, 2D=0x02, 3D=0x03,
                  // deadreck+GPS=0x04, time fix only=0x05, reserved=0x06..0xff
  uint8_t flags;  //!< Fix Status Flags Bitfield
                  //!< (0-GPSFixOk,1-DiffSoln,2-WKNSet,3-TOWSet)
  uint8_t  fixstat;
  uint8_t  flags2;  // bits 0-1 : psmState
                    // bits 3-4 : spoofDetState
                      // 0: Unknown or deactivated, 
                      // 1: No spoofing indicated, 
                      // 2: Spoofing indicated, 
                      // 3: Multiple spoofing indications
                    // bist 6-7 : carSolN
  uint32_t ttff;  // TTFF (ms)
  uint32_t msss;  // Milliseconds since startup/reset
  uint8_t  checksum[2];
});

/*!
 * NAV-SOL Message Structure
 * This message combines Position, velocity and
 * time solution in ECEF, including accuracy figures.
 * ID: 0x01  0x06  Payload Length=52 bytes
 */

static const uint16_t NAVSOL_FLAG_GPSFIX_VALID      = 1 << 0;
static const uint16_t NAVSOL_FLAG_DGPS_USED_FOR_FIX = 1 << 1;
static const uint16_t NAVSOL_FLAG_WEEK_NUM_VALID    = 1 << 2;
static const uint16_t NAVSOL_FLAG_TOW_VALID         = 1 << 3;

PACK(struct NavSol {
  UbloxHeader header;
  uint32_t    iTOW;    //!< Gps time of week [msec]
  int32_t     fTOW;    //!< Fractional remainder of iTOW [nsec]
  int16_t     week;    //!< GPS week
  uint8_t     gpsFix;  //!< GpsFix Type (0-5)
  int8_t      flags;   //!< Fix Status Flags Bitfield
                       //!< (0-GPSFixOk,1-DiffSoln,2-WKNSet,3-TOWSet)
  int32_t  ecefX;      //!< ECEF X coordinate [cm]
  int32_t  ecefY;      //!< ECEF Y coordinate [cm]
  int32_t  ecefZ;      //!< ECEF Z coordinate [cm]
  uint32_t pAcc;       //!< 3D position accuracy estimate (std) [cm]
  int32_t  ecefVX;     //!< ECEF X velocity [cm/sec]
  int32_t  ecefVY;     //!< ECEF Y velocity [cm/sec]
  int32_t  ecefVZ;     //!< ECEF Z velocity [cm/sec]
  uint32_t sAcc;       //!< Speed accuracy estimate (std) [cm/sec]
  uint16_t pDop;       //!< Position DOP, scale factor *0.01
  uint8_t  reserved1;
  uint8_t  numSV;  //!< Number of SVs used in nav solution
  uint32_t reserved2;
  uint8_t  checksum[2];
});

/*!
 * NAV-PVT Message Structure
 * This message combines Position, velocity, and
 * time solution in LLH, including accuracy figures.
 * ID: 0x01  0x07  Payload Length=92 bytes
 */
PACK(struct NavPvt {
  UbloxHeader header;
  uint32_t    iTOW;                   //!< Gps time of week [msec]
  uint16_t    year;                   //!< Year (UTC)
  uint8_t     month;                  //!< Month {1..12} (UTC)
  uint8_t     day;                    //!< Day of month {1..31} (UTC)
  uint8_t     hour;                   //!< Hour of day {0..23} (UTC)
  uint8_t     min;                    //!< Minute of hour {0..59}
  uint8_t     sec;                    //!< Seconds of minutes {0..60} (UTC)
  uint8_t     validity_flags;         //!< Validy flags
  uint32_t    time_accuracy;          //<! Time accuracy [ns] (UTC)
  int32_t     nano;                   //!< Fractional remainder of iTOW [nsec]
  uint8_t     gpsFix;                 //!< GpsFix Type (0-5)
  uint8_t     flags;                  //!< Fix Status Flags
  uint8_t     additional_flags;       //!< Additional Flags
  uint8_t     numSV;                  //!< Number of SVs used in nav solution
  int32_t     longitude_scaled;       //!< longitude in degrees. Scaling 1e-7
  int32_t     latitude_scaled;        //!< latitude in degrees. Scaling 1e-7
  int32_t     height;                 //!< height above ellipsoid [mm]
  int32_t     height_mean_sea_level;  //!< height above mean sea level [mm]
  uint32_t    horizontal_accuracy;    //!< horizontal accuracy estimate [mm]
  uint32_t    vertical_accuracy;      //!< vertical accuracy estimate [mm]
  int32_t     velocity_north;         //!< north velocity [mm/s]
  int32_t     velocity_east;          //!< east velocity [mm/s]
  int32_t     velocity_down;          //!< down velocity [mm/s]
  int32_t     ground_speed;           //!< 2D (ground) speed [mm/s]
  int32_t     heading_motion;         //!< heading of motion[deg]. Scaling 1e-5
  uint32_t    speed_accuracy;         //!< speed accuracy estimate [mm/s]
  uint32_t
           heading_accuracy;  //!< course/heading accuracy estimate [deg]. Scaling 1e-5
  uint16_t p_dop;             //!< Position DOP, scale factor *0.01
  uint8_t  reserved[6];      //!< Reserved
  int32_t  heading_vehicle;  //!< Heading of vehicle [deg], scale factor 1e-5
  int16_t
    magnetic_declination;  //!< Magnetic declination [deg], scale factor 1e-2
  uint16_t
          mag_dec_acc;  //!< Magnetic declination accuracy [deg], scale factor 1e-2
  uint8_t checksum[2];
});

/*!
 * NAV-POSECEF Message Structure
 * This message outputs the Geodetic position in ECEF coordinates
 * ID: 0x01  0x01  Payload Length=20 bytes
 */
PACK(struct NavPosECEF {
  UbloxHeader header;  //!< Ublox header
  uint32_t    iTOW;    //!< GPS millisecond time of week
  int32_t     x;       //!< ECEF X coordinate [cm]
  int32_t     y;       //!< ECEF Y coordinate [cm]
  int32_t     z;       //!< ECEF Z coordinate [cm]
  uint32_t    posAcc;  //!< Position accuracy estimate [cm]
  uint8_t     checksum[2];
});

/*!
 * NAV-POSLLH Message Structure
 * This message outputs the Geodetic position in
 * the currently selected Ellipsoid. The default is
 * the WGS84 Ellipsoid, but can be changed with the
 * message CFG-DAT.
 * ID: 0x01  0x02  Payload Length=28 bytes
 */
PACK(struct NavPosLLH {
  UbloxHeader header;                 //!< Ublox header
  uint32_t    iTOW;                   //!< GPS millisecond time of week
  int32_t     longitude_scaled;       //!< longitude in degrees. Scaling 1e-7
  int32_t     latitude_scaled;        //!< latitude in degrees. Scaling 1e-7
  int32_t     height;                 //!< height above ellipsoid [mm]
  int32_t     height_mean_sea_level;  //!< height above mean sea level [mm]
  uint32_t    horizontal_accuracy;    //!< horizontal accuracy estimate [mm]
  uint32_t    vertical_accuracy;      //!< vertical accuracy estimate [mm]
  uint8_t     checksum[2];
});

/*!
 * NAV-VELNED Message Structure
 * This message outputs the current 3D velocity
 * in a north-east-down frame.
 * ID: 0x01  0x12  Payload Length=36 bytes
 */
PACK(struct NavVelNed {
  UbloxHeader header;          //!< Ublox header
  uint32_t    iTOW;            //!< GPS Time of week [msec]
  int32_t     velocity_north;  //!< north velocity [cm/s]
  int32_t     velocity_east;   //!< east velocity [cm/s]
  int32_t     velocity_down;   //!< down velocity [cm/s]
  uint32_t    speed;           //!< 3D speed [cm/s]
  uint32_t    ground_speed;    //!< 2D (ground) speed [cm/s]
  int32_t     heading_scaled;  //!< heading [deg]. Scaling 1e-5
  uint32_t    speed_accuracy;  //!< speed accuracy estimate [cm/s]
  uint32_t
          heading_accuracy;  //!< course/heading accuracy estimate [deg]. Scaling 1e-5
  uint8_t checksum[2];
});

/*!
 * NAV-SVINFO Message Structure
 * This message outputs info about SVs each
 * channel is tracking
 * ID: 0x01  0x30  Payload Length= (8+12*NumChannels bytes)
 */
PACK(struct SVInfoReapBlock {
  uint8_t ch_num;   //!< Channel Number (255 if SV isn't assigned to channel)
  uint8_t svid;     // Satellite ID number
  uint8_t flags;    // bitfield (description of contents follows)
  uint8_t quality;  // signal quality indicator bitfield
  uint8_t cno;      // carrier to noise ratio (dbHz)
  int8_t  elev;     // elevation (deg)
  int16_t azim;     // azimuth (deg)
  int32_t prRes;    // Psuedorange residual (centimeters)
});

PACK(struct NavSVInfo {
  UbloxHeader header;        //!< Ublox header
  uint32_t    iTOW;          // GPS time of week (ms)
  uint8_t     numch;         //! number of channels following
  uint8_t     global_flags;  // Chip and Hardware Generation
  uint16_t    reserved2;
  SVInfoReapBlock
          svinfo_reap[MAXCHAN];  // NOTE: TODO: True max needs to be confirmed
  uint8_t checksum[2];
});
// Description of flags bitfield
static const uint32_t NAV_SVINFO_FLAGS_USED4NAV = 1 << 0;  // SV used in NAV sol
static const uint32_t NAV_SVINFO_FLAGS_DGPS_AVAIL =
  1 << 1;  // DGPS corr data available for SV
static const uint32_t NAV_SVINFO_FLAGS_ORBIT_INFO_AVAIL =
  1 << 2;  // Ephemeris of Almanac orbit info available for SV
static const uint32_t NAV_SVINFO_FLAGS_EPHEMS_AVAIL =
  1 << 3;  // Ephemeris orbit info available for SV
static const uint32_t NAV_SVINFO_FLAGS_SV_UNHEALTHY =
  1 << 4;  // SV unhealthy and not used
static const uint32_t NAV_SVINFO_FLAGS_ALMPLUS_AVAIL =
  1 << 5;  // Almanac Plus orbit info used
static const uint32_t NAV_SVINFO_FLAGS_ASSNOW_AUTO =
  1 << 6;  // AssistNow Autonomous orbit info used
static const uint32_t NAV_SVINFO_FLAGS_PR_SMOOTHED =
  1 << 7;  // Carrier Smoothed pseudorange used (PPP)

/*!
 * NAV-SAT Message Structure
 * This message displays information about SVs which are either known to be
 * visible or currently tracked by the receiver.
 * ID: 0x01  0x35 Payload Length= (8+12*NumChannels bytes)
 */

struct NavSatFlags
{
  uint32_t qualityInd : 3;
  uint32_t svUsed : 1;
  uint32_t health : 2;
  uint32_t diffCorr : 1;
  uint32_t orbitSource : 3;
  uint32_t ephemAvail : 1;
  uint32_t almAvail : 1;
  uint32_t anoAvail : 1;
  uint32_t aopAvail : 1;
  uint32_t reserved1 : 1;
  uint32_t sbasCorrUsed : 1;
  uint32_t rtcmCorrUsed : 1;
  uint32_t slasCorrUsed : 1;
  uint32_t reserved2 : 1;
  uint32_t prCorrUsed : 1;
  uint32_t crcorrUsed : 1;
  uint32_t doCorrUsed : 1;
  uint32_t reserved3 : 9;
};

PACK(struct NavSatBlock {
  GnssId      gnssId;   //!< GNSS Identified
  uint8_t     svId;     //!< Satellite ID number
  uint8_t     cN0;      //!< carrier to noise ratio (dbHz)
  int8_t      elev;     //!< elevation (deg)
  int16_t     azim;     //!< azimuth (deg)
  int16_t     prResCm;  //!< Psuedorange residual [m] (Scaling = 0.1);
  NavSatFlags flags;    //!< bitfield (description of contents follows)
});

PACK(struct NavSat {
  UbloxHeader header;   //!< Ublox header
  uint32_t    iTOW;     //!< GPS time of week (ms)
  uint8_t     version;  //!< Message version (1)
  uint8_t     numSvs;   //! number of satellites following
  uint16_t    reserved;
  NavSatBlock satInfo[MAX_SAT];  // TODO: True max needs to be confirmed
  uint8_t     checksum[2];
});

/*!
 * NAV-SIG Message Structure
 * This message displays information about signals currently tracked by the
 * receiver.
 * ID: 0x01  0x35 Payload Length= (8+12*NumChannels bytes)
 */
struct NavSigFlags
{
  uint16_t health : 2;
  uint16_t prSmoothed : 1;
  uint16_t prUsed : 1;
  uint16_t crUsed : 1;
  uint16_t doUsed : 1;
  uint16_t prCorrUsed : 1;
  uint16_t crCorrUsed : 1;
  uint16_t doCorrUsed : 1;
  uint16_t reserved : 7;
};

PACK(struct NavSigBlock {
  GnssId           gnssId;      //!< GNSS Identified
  uint8_t          svId;        //!< Satellite ID number
  uint8_t          sigId;       //!< Signal identifier
  uint8_t          freqId;      //!< Frequency identifier
  int16_t          psrResCm;    //!< Pseudorange residual [cm]
  uint8_t          cN0;         //!< carrier to noise ratio (dbHz)
  QualityIndicator quality;     //!< Quality indicator
  CorrectionSource corrSource;  //!< Correction source
  IonoModel        ionoModel;   //!< Iono model
  NavSigFlags      flags;       //!< bitfield (description of contents follows)
  uint32_t         reserved2;
});

PACK(struct NavSig {
  UbloxHeader header;   //!< Ublox header
  uint32_t    iTOW;     //!< GPS time of week (ms)
  uint8_t     version;  //!< Message version (1)
  uint8_t     numSigs;  //! number of satellites following
  uint16_t    reserved;
  NavSigBlock sigInfo[MAX_SIG];  // TODO: True max needs to be confirmed
  uint8_t     checksum[2];
});

/*!
 * NAV-TIMEGPS Message Structure
 * This message outputs GPS Time information
 * ID: 0x01  0x20  Payload Length= 16 bytes
 */
PACK(struct NavGPSTime {
  UbloxHeader header;
  uint32_t    iTOW;      // GPS ms time of week
  int32_t     ftow;      // fractional nanoseconds remainder
  int16_t     week;      // GPS week
  int8_t      leapsecs;  // GPS UTC leap seconds
  uint8_t     valid;     // validity flags
  uint32_t    tacc;      // time accuracy measurement (nanosecs)
  uint8_t     checksum[2];
});

/*!
 * NAV-TIMEUTC Message Structure
 * This message outputs UTC Time information
 * ID: 0x01  0x21  Payload Length= 20 bytes
 */
PACK(struct NavUTCTime {
  UbloxHeader header;
  uint32_t    iTOW;   // GPS time of week (msec)
  uint32_t    tacc;   // time accuracy measurement
  int32_t     nano;   // Nanoseconds of second
  uint16_t    year;   // year
  uint8_t     month;  // month
  uint8_t     day;    // day
  uint8_t     hour;   // hour
  uint8_t     min;    // minute
  uint8_t     sec;    // second
  uint8_t     valid;  // validity flags
  uint8_t     checksum[2];
});

/*!
 * NAV-DOP Message Structure
 * This message outputs various DOPs. All
 * DOP values are scaled by a factor of 100.
 * Ex. If gdop contains a value 156, the true
 * value is 1.56
 * ID: 0x01  0x04  Payload Length= 18 bytes
 */
PACK(struct NavDOP {
  UbloxHeader header;
  uint32_t    iTOW;  // GPS ms time of week (ms)
  uint16_t    gdop;  // Geometric DOP
  uint16_t    pdop;  // Position DOP
  uint16_t    tdop;  // Time DOP
  uint16_t    vdop;  // Vertical DOP
  uint16_t    hdop;  // Horizontal DOP
  uint16_t    ndop;  // Northing DOP
  uint16_t    edop;  // Easting DOP
  uint8_t     checksum[2];
});

/*!
 * NAV-DGPS Message Structure
 * This message outputs DGPS correction data as it
 * has been applied to the current NAV Solution
 * ID: 0x01  0x31  Payload Length= (16 + 12*numChannels bytes)
 */
PACK(struct NavDGPSReap {
  uint8_t  svid;
  uint8_t  flags;  // bitfield containing channel each sv is on and DGPS status
  uint16_t agecorr;  // age of latest correction data (ms)
  float    prcorr;   // psuedorange correction   (m)
  float    prrcorr;  // psuedorange rate correction (m/sec)
});

PACK(struct NavDGPS {
  UbloxHeader header;
  uint32_t    iTOW;        // GPS ms time of week
  int32_t     age;         // age of newest correction data (ms)
  int16_t     baseID;      // DGPS base station ID
  int16_t     basehealth;  // DGPS base station health
  uint8_t numchan;  // nomber of channels for which correction data is following
  uint8_t status;   // DGPS correction type status
  uint16_t    reserved;       // reserved
  NavDGPSReap nav_dgps_reap;  // repeated portion of NAV-DGPS message
  uint8_t     checksum[2];
});

/*!
 * NAV-CLOCK Message Structure
 * This message outputs receiver clock information
 * ID: 0x01  0x22  Payload Length= 20 bytes
 */
PACK(struct NavClock {
  UbloxHeader header;
  uint32_t    iTOW;      // GPS time of week (msec)
  int32_t     clkbias;   // clock bias in nanoseconds
  int32_t     clkdrift;  // clock drift in ns/s
  uint32_t    tacc;      // time accuracy estimate (ns)
  uint32_t    facc;      // frequency accuracy estimate (ps/s)
  uint8_t     checksum[2];
});

//////////////////////////////////////////////////////////////
// AIDING DATA MESSAGES
//////////////////////////////////////////////////////////////
/*!
 * AID-INI Message Structure
 * Reciever Position, Time, Clock Drift, Frequency
 * ID: 0x0B  0x01 Payload Length=48 bytes
 */
static const uint8_t PAYLOAD_LENGTH_AID_INI = 48;
static const uint8_t FULL_LENGTH_AID_INI    = (48 + 8);
PACK(struct AidIni {
  UbloxHeader header;      //!< Ublox header
  int32_t     ecefXorLat;  //!< ECEF x position or latitude [cm or deg*1e-7]
  int32_t     ecefYorLon;  //!< ECEF y position or longitude [cm or deg*1e-7]
  int32_t     ecefZorAlt;  //!< ECEF z position or altitude [cm]
  uint32_t    position_accuracy;   //!< position accuracy - std dev [cm]
  uint16_t    time_configuration;  //!< time configuration bit misk
  uint16_t    week_number;         //!< actual week number
  uint32_t    time_of_week;        //!< actual time of week [ms]
  int32_t     time_of_week_ns;     //!< fractional part of time of week [ns]
  uint32_t    time_accuracy_ms;    //!< time accuracy [ms]
  uint32_t    time_accuracy_ns;    //!< time accuracy [ns]
  int32_t  clock_drift_or_freq;  //!< clock drift or frequency [ns/s or Hz*1e-2]
  uint32_t clock_drift_or_freq_accuracy;  //!< clock drift or frequency accuracy
                                          //[ns/s or ppb]
  uint32_t flags;  //!< bit field that determines contents of other fields
  uint8_t  checksum[2];
});

// defines for AidIni flags
static const uint16_t AIDINI_FLAG_POSITION_VALID    = 0x01;
static const uint16_t AIDINI_FLAG_TIME_VALID        = 0x02;
static const uint16_t AIDINI_FLAG_CLOCK_DRIFT_VALID = 0x04;
static const uint16_t AIDINI_FLAG_USE_TIME_PULSE    = 0X08;
static const uint16_t AIDINI_FLAG_CLOCK_FREQ_VALID  = 0x10;
static const uint16_t AIDINI_FLAG_USE_LLA           = 0x20;
static const uint16_t AIDINI_FLAG_ALTITUDE_INVALID  = 0X40;
static const uint16_t AIDINI_FLAG_UTC_TIME          = 0x0400;
static const uint16_t AIDINI_USE_PREV_TIME_PULSE    = 0X80;

/*!
 * AID-HUI Message Structure
 * GPS Health, Ionospheric, and UTC Parameters
 * ID: 0x0B  0x02 Payload Length: 72
 */
static const uint8_t PAYLOAD_LENGTH_AID_HUI = 72;
static const uint8_t FULL_LENGTH_AID_HUI    = (72 + 8);
PACK(struct AidHui {
  UbloxHeader header;
  uint32_t    health;
  double      a0;
  double      a1;
  uint32_t    tow;
  int16_t     week;
  int16_t     beforeleapsecs;
  int16_t     nextleapsecweek;
  int16_t     nextleapsec;
  int16_t     afterleapsecs;
  int16_t     spare;
  float       kloba0;
  float       kloba1;
  float       kloba2;
  float       kloba3;
  float       klobb0;
  float       klobb1;
  float       klobb2;
  float       klobb3;
  uint32_t    flags;
  uint8_t     checksum[2];
});
// defines for AID-HUI flags
static const uint8_t AIDHUI_FLAG_HEALTH_VALID = 1 << 0;
static const uint8_t AIDHUI_FLAG_UTC_VALID    = 1 << 1;
static const uint8_t AIDHUI_FLAG_KLOB_VALID   = 1 << 2;

/*!
 * AID-EPH Message Structure
 * This message contains ephemeris for a satellite.
 * ID: 0x0B 0x31 Payload Length = (16) or (112) bytes
 */
static const uint8_t PAYLOAD_LENGTH_AID_EPH_WITH_DATA = 104;
static const uint8_t PAYLOAD_LENGTH_AID_EPH_NO_DATA   = 8;
static const uint8_t FULL_LENGTH_AID_EPH_WITH_DATA    = (104 + 8);
static const uint8_t FULL_LENGTH_AID_EPH_NO_DATA      = (8 + 8);
PACK(struct EphemW {
  uint8_t byte[4];  // Each Word contains 4 bytes (4th is ignored)
});

PACK(struct EphemSF {
  // uint32_t W[8];				// Words 3-10 of Subframes
  EphemW W[8];
});

PACK(struct EphemSV {       // Ephemeris for a Satellite
  UbloxHeader header;       // Header
  uint32_t    svprn;        // Satellite Number
  uint32_t    HOW;          // Hand Over Word
  EphemSF     SF[3];        // Subframes
  uint8_t     checksum[2];  // Checksum
});

PACK(struct Ephemerides {  // Holds EphemSV message for all SVs
  EphemSV ephemsv[MAX_SAT];
});

// Parsed Ephemeris Parameters for a SV - NOT FINISHED
PACK(struct ParsedEphemData {
  uint32_t prn;  // PRN number
  uint8_t  tow;  // time stamp of subframe 0 (s)
  // uint8_t tow;				//time stamp of subframe 0 (s)
  unsigned long health;   // health status, defined in ICD-GPS-200
  unsigned long iode1;    // issue of ephemeris data 1
  unsigned long iode2;    // issue of ephemeris data 2
  unsigned long week;     // GPS week number
  unsigned long zweek;    // z count week number
  double        toe;      // reference time for ephemeris (s)
  double        majaxis;  // semi major axis (m)
  double        dN;       // Mean motion difference (rad/s)
  double        anrtime;  // mean anomoly reference time (rad)
  double        ecc;      // eccentricity
  double        omega;    // arguement of perigee (rad)
  double        cuc;      // arugument of latitude - cos (rad)
  double        cus;      // argument of latitude - sine (rad)
  double        crc;      // orbit radius - cos (rad)
  double        crs;      // orbit radius - sine (rad)
  double        cic;      // inclination - cos (rad)
  double        cis;      // inclination - sine (rad)
  double        ia;       // inclination angle (rad)
  double        dia;      // rate of inclination angle (rad/s)
  double        wo;       // right ascension (rad)
  double        dwo;      // rate of right ascension (rad/s)
  unsigned long iodc;     // issue of data clock
  double        toc;      // SV clock correction term (s)
  double        tgd;      // estimated group delay difference
  double        af0;      // clock aiging parameter 0
  double        af1;      // clock aiging parameter 1
  double        af2;      // clock aiging parameter 2
  //      yes_no spoof;			//anti spoofing on
  double       cmot;  // corrected mean motion
  unsigned int ura;   // user range accuracy variance (value 0-15)
});

// Contains Ephemeris Parameters for all SVs
PACK(struct ParsedEphemeridesData { ParsedEphemData sv_eph_data[MAX_SAT]; });

/*!
 * AID-ALM Message Structure
 * This message contains GPS almanac data for a satellite
 * ID: 0x0B 0x30 Payload Length = (8) or (48) bytes
 */
static const uint8_t PAYLOAD_LENGTH_AID_ALM_WITH_DATA = 40;
static const uint8_t PAYLOAD_LENGTH_AID_ALM_NO_DATA   = 8;
static const uint8_t FULL_LENGTH_AID_ALM_WITH_DATA    = (40 + 8);
static const uint8_t FULL_LENGTH_AID_ALM_NO_DATA      = (8 + 8);

PACK(struct AlmW {
  uint8_t byte[4];  // Each Word contains 4 bytes (4th is ignored)
});

PACK(struct AlmSV {
  UbloxHeader header;       // Header
  uint32_t    svprn;        // Satellite Number
  uint32_t    issue_week;   // Issue date of Almanac
  AlmW        words[8];     // Words 3-10 of Almanac data for an SV
  uint8_t     checksum[2];  // Checksum
});

// Holds Almanac data for all SVs
PACK(struct Almanac { AlmSV almsv[MAX_SAT]; });

/*!
 * RXM-RAW Message Structure
 * This message contains raw DGPS measurements data
 * ID: 0x02 0x10 Payload Length = (8 + 24*#SVs) bytes
 */
//#define RXMRAW_QUALITY_PR_DOP_GOOD 4 // Min value for pseudorange and doppler
// to be good
//#define RXMRAW_QUALITY_PR_DOP_CP_GOOD 4 // Min value for pseudorange, doppler,
// and carrier phase to be good

enum class LossOfLockMask
{                            // Follows RINEX definition
  NoSlipDetected    = 0x00,  // 0b00
  PossibleCycleSlip = 0x01,  // 0b01 Lost lock between prev and current interval
  PossibleHalfCycleSlip = 0x02  // 0b01 Valid for current epoch only

};

PACK(struct RawMeasReap {
  double  carrier_phase;  // cycles - Carrier Phase measurement
  double  psuedorange;    // m - Psuedorange measurement
  float   doppler;        // Hz - Doppler Measurement
  uint8_t svid;           // SV Number
  int8_t  quality;  // Nav Measurement Quality Indicator  -- (>=4 PR+DO OK) (>=5
                    // PR+DO+CP OK) (<6 likel loss carrier lock)
  int8_t  cno;      // dbHz - Carrier to Noise Ratio
  uint8_t loss_of_lock_indicator;  // Loss of Lock Indicator (RINEX Definition)
});

PACK(struct RawMeas {
  UbloxHeader header;
  int32_t     iTow;   // ms - Time of Week
  int16_t     week;   // weeks
  uint8_t     numSV;  // # of SVs following
  uint8_t     reserved;
  RawMeasReap rawmeasreap[MAX_SAT];
  uint8_t     checksum[2];
});

/*!
 * RXM-RAWX Message Structure - Protocol Version 17
 * This message contains raw DGPS measurements data
 * ID: 0x02 0x15 Payload Length = (16 + 32*#SVs) bytes
 */

PACK(struct RawxMeasReap {
  double   psuedorange;    // m - Psuedorange measurement
  double   carrier_phase;  // cycles - Carrier Phase measurement
  float    doppler;        // Hz - Doppler Measurement
  GnssId   gnssId;         // GNSS identifier
  uint8_t  svid;           // SV Number
  uint8_t  sigId;          // New style signal identifier
  uint8_t  freqId;         // GLONASS frequency slot
  uint16_t locktime;       // ms - carrier phase lock time (max 64500ms)
  uint8_t  cno;            // dbHz - Carrier to Noise Ratio
  uint8_t  psrStdDev;  // m - estimated pseudorange standard deviation - scaling
                       // 0.01*2^n
  uint8_t cpStdDeve;   // cycles - estimated carrier phase standard deviation -
                       // scaling 0.004
  uint8_t
          doStdDev;  // Hz - estimated doppler standard deviation - scaling 0.002*2^n
  uint8_t trackingStat;  // Tracking status bitfield
  uint8_t reserved3;
});

PACK(struct RawMeasX {
  UbloxHeader  header;
  double       rcvTow;       // Measurement time of week in GPS seconds
  uint16_t     week;         // GPS week
  int8_t       leepSeconds;  // GPS leap seconds
  uint8_t      numMeas;      // number of measurements
  uint8_t      recvStatus;   // receiver tracking status
  uint8_t      reserved1[3];
  RawxMeasReap rawxmeasreap[MAX_SAT];
  uint8_t      checksum[2];
});

/*!
 * RXM-SFRB Message Structure
 * This message contains the contents of a single subframe
 * w/ parity bits removed
 * ID: 0x02 0x11 Payload Length = (42) bytes
 */

PACK(struct SubframeData {
  UbloxHeader header;
  uint8_t     chan;       // Channel Number
  uint8_t     svid;       // Satellite ID Number
  int32_t     words[10];  // Words of data
  /*
  Each word contains 24 bits of data (Bits 23 to 0).  The remaining 8
  bits are undefined.  The direction within the Word is that the higher
  order bits are received from the SV first.
  Example:
      The Preamble can be found in dwrd[0], at bit position 23 down to 16.
  */
  uint8_t checksum[2];
});

/*!
 * RXM-SFRBX Message Structure
 * This message reports a complete subframe of broadcast navigation data
 * decoded from a single signal. The number of data words reported in each
 * message depends on the nature of the signal.
 * ID: 0x02 0x13 Payload Length = (8+4*numWords) bytes
 */

PACK(struct SubframeDataX {
  UbloxHeader header;
  GnssId     gnssId;
  uint8_t     svID;
  uint8_t     reserved1;
  uint8_t     freqId;
  uint8_t     numWords;
  uint8_t     chn;
  uint8_t     version;
  uint8_t     reserved2;
  uint32_t    dwrd[10]; // Max is 10, from ZED-F9P Interface Description
  uint8_t checksum[2];
});
/*!
 * RXM-SVSI Message Structure
 * This message contains SV orbit knowledge for SVs
 * ID: 0x02 0x20 Payload Length = (8 + 6*#SVs) bytes
 */
PACK(struct SVStatusReap {
  uint8_t svid;    // Satellite ID
  uint8_t svflag;  // Information Flag
  int16_t azim;    // Azimuth
  int8_t  elev;    // Elevation
  uint8_t age;     // Age of almanac and ephemeris
});

PACK(struct SVStatus {
  UbloxHeader  header;
  int32_t      iTow;               // ms - Time of Week
  int16_t      week;               // weeks - GPS Week
  uint8_t      numvis;             // Number of visible SVs
  uint8_t      numSV;              // # of SVs following
  SVStatusReap svstatusreap[100];  // NOTE: TODO: Find the max repititions
                                   // possible for this!! max thus far: (71)
  uint8_t checksum[2];
});

//////////////////////////////////////////////////////////////
//!< Monitoring Messages (MON)
//////////////////////////////////////////////////////////////

enum class JammingState : uint8_t
{
  UNKNOWN  = 0,  //!< Unknown or feature disabled
  OK       = 1,  //!< OK. No significant jamming
  WARNING  = 2,  //!< Warning - Interference visible but fix OK
  CRITICAL = 3   //!< Critical - Interference visible and no fix
};

enum class AntennaStatus : uint8_t
{
  INIT     = 0,
  DONTKNOW = 1,
  OK       = 2,
  SHORT    = 3,
  OPEN     = 4
};

enum class AntennaPower : uint8_t
{
  OFF      = 0,
  ON       = 1,
  DONTKNOW = 2
};

/*!
 * MON-RF-Block Message Structure
 * Repeated blocks used in MON-RF messages
 */
PACK(struct MonRfBlock {
  uint8_t       blockId;        //!< RF block id
  JammingState  jammingState;   //!< Jamming flags
  AntennaStatus antennaStatus;  //!< Status of antenna supervisor
  AntennaPower  antennaPower;   //!< Power status of the antenna
  uint32_t      postStatus;     //!< Post status word
  uint8_t       reserved2[4];   //!< Reserved
  uint16_t      noisePerMS;     //!< Noise level as measured by the GPS core
  uint16_t agcCnt;  //!< AGC monitor (counts SIGHI xor SIGLO, range 0 to 8191)
  int8_t jammingIndicator;  //!< CW jamming indicator, scaled (0=no CW jamming,
                            //!< 255 = strong CW jamming)
  int8_t ofsI;   //!< Imbalance of I-part of complex signal, scaled (-128 = max
                 //!< neg imbalance, 127 = max pos imbalance)
  uint8_t magI;  //!< Magnitude of I-part of complex signal, scaled (0 = no
                 //!< signal, 255 = max magnitude)
  int8_t ofsQ;   //!< Imbalance of Q-part of complex signal, scaled (-128 = max
                 //!< neg imbalance, 127 = max pos imbalance)
  uint8_t magQ;  //!< Magnitude of Q-part of complex signal, scaled (0 = no
                 //!< signal, 255 = max magnitude)
  uint8_t reserved3[3];  //!< Reserved
});

/*!
 * MON-RF Message Structure
 * RF hardware status including AGC and jamming indication
 * ID: 0x0A  0x38  Payload Length=4 + 24*nBlocks bytes
 */
PACK(struct MonRf {
  UbloxHeader header;
  uint8_t     version;    //!< Message version (0x00 for this version)
  uint8_t     numBlocks;  //!< Number of RF blocks in message
  MonRfBlock  rfBlocks[MAX_BLOCKS];  //!< RF blocks
  uint8_t     checksum[2];
});

/*!
 * MON-GNSS Message Structure
 * This message extended hardware status information
 * ID: 0x0A  0x28  Payload Length= 8 bytes
 */
PACK(struct MonGnss {
  UbloxHeader header;
  uint8_t     version;  //!< Message version
  int8_t
         supported;  //!< A bit mask showing the major GNSS that can be supported
  int8_t defuaultGnss;  //!< A bit mask showing the default major GNSS selection
  int8_t enabled;       //!< A bitmsak showing the current major GNSS selection
                        //!< enabled for this reciever
  uint8_t simultaneous;  //!< Maximum number of concurrent major GNSS that can
                         //!< be supported by this reciever
  uint8_t reserved1[3];  //!< Reserved
  uint8_t checksum[2];
});

static const uint32_t MONGNSS_FLAG_GPS_ENABLE      = 1 << 0;
static const uint32_t MONGNSS_FLAG_GLONASS_ENABLE  = 1 << 1;
static const uint32_t MONGNSS_FLAG_BEIDOU_ENABLE   = 1 << 2;
static const uint32_t MONGNSS_FLAG_GALILIEO_ENABLE = 1 << 4;

/*!
 * MON-HW2 Message Structure
 * This message extended hardware status information
 * ID: 0x0A  0x0B  Payload Length= 28 bytes
 */

PACK(struct MonHw2 {
  UbloxHeader header;
  int8_t ofsI;   //!< Imbalance of I-part of complex signal, scaled (-128 = max
                 //!< neg imbalance, 127 = max pos imbalance)
  uint8_t magI;  //!< Magnitude of I-part of complex signal, scaled (0 = no
                 //!< signal, 255 = max magnitude)
  int8_t ofsQ;   //!< Imbalance of Q-part of complex signal, scaled (-128 = max
                 //!< neg imbalance, 127 = max pos imbalance)
  uint8_t magQ;  //!< Magnitude of Q-part of complex signal, scaled (0 = no
                 //!< signal, 255 = max magnitude)
  uint8_t cfgSource;  //!< Source of low-level configuration (114 = ROM, 111 =
                      //!< OTP, 112 = config pins, 102 = flash image)
  uint8_t  reserved1[3];  //!< Reserved
  uint32_t lowLevCfg;     //!< Low-level configuration
  uint8_t  reserved2[8];  //!< Reserved
  uint32_t postStatus;    //!< POST status word
  uint8_t  reserved3[4];  //!< Reserved
  uint8_t  checksum[2];
});

/*!
 * MON-HW Message Structure
 * This message hardware status information
 * ID: 0x0A  0x09  Payload Length= 60 bytes
 */
PACK(struct MonHw {
  UbloxHeader header;
  int32_t     pinSel;      //!< Mask of Pins Set as Peripheral/PIO
  int32_t     pinBank;     //!< Mask of Pins Set as Bank A/B
  int32_t     pinDir;      //!< Mask of Pins Set as Input/Output
  int32_t     pinVal;      //!< Mask of Pins Value Low/High
  uint16_t    noisePerMS;  //!< Noise Level as measured by the GPS Core
  uint16_t agcCnt;  //!< AGC Monitor (counts SIGHI xor SIGLO, range 0 to 8191)
  uint8_t aStatus;  //!< Status of the Antenna Supervisor State Machine (0=INIT,
                    //!< 1=DONTKNOW, 2=OK, 3=SHORT, 4=OPEN)
  uint8_t aPower;  //!< Current PowerStatus of Antenna (0=OFF, 1=ON, 2=DONTKNOW)
  uint8_t flags;   //!< Flags (see graphic below)
  uint8_t reserved1;  //!< Reserved
  int32_t usedMask;   //!< Mask of Pins that are used by the Virtual Pin Manager
  uint8_t VP[17];  //!< Array of Pin Mappings for each of the 17 Physical Pins
  uint8_t jamInd;  //!< CW Jamming indicator, scaled (0 = CW jamming, 255 =
                   //!< strong CW jamming)
  uint8_t reserved2[2];  //!< Reserved
  int32_t pinIrq;        //!< Mask of Pins Value using the PIO Irq
  int32_t pullH;  //!< Mask of Pins Value using the PIO Pull High Resistor
  int32_t pullL;  //!< Mask of Pins Value using the PIO Pull Low Resistor
  uint8_t checksum[2];
});

/*!
 * MON-VER Message Structure
 * This message receiver/Software version information
 * ID: 0x0A  0x04  Payload Length= (40 + 30*N) bytes
 */

PACK(struct MonVerReap {
  int8_t extension[30];  //!< Extended software information strings.
});

PACK(struct MonVer {
  UbloxHeader header;
  int8_t      swVersion[30];  //!< Zero-terminated Software Version String
  int8_t      hwVersion[10];  //!< Zero-terminated Hardware Version String
  MonVerReap  mon_ver_reap;
  uint8_t     checksum[2];
});

/*!
 * MON-SPAN Message Structure
 * Basic spectrum analyzer. One spectrum display for each RF path
 * ID: 0x0A  0x31  Payload Length= (4 + 272*N) bytes
 */

// The center frequency at each bin, assuming a zero based bin count,
//    center_freq(i) = center + span * (i*128)/256
PACK(struct MonSpanData {
  uint8_t
           spectrum[256];  //!< Spectrum data (number of points = span/resolution)
  uint32_t span;           //!< Spectrum span (Hz)
  uint32_t resolution;     //!< Resolution of the spectrum (Hz)
  uint32_t center;         //!< Center of spectrum span (Hz)
  uint8_t  pga;            //!< Programmagle gain amplifier (dB)
  uint8_t  reserved1[3];  //!< Reserved
});

struct MonSpan
{
  UbloxHeader              header;
  uint8_t                  version;       //!< Message version
  uint8_t                  numRfBlocks;   //!< Number of RF blocks following
  uint8_t                  reserved0[2];  //!< Reserved
  std::vector<MonSpanData> data;          //!< Repeated group of spectrum data
  uint8_t                  checksum[2];
};

// typedef enum BINARY_LOG_TYPE BINARY_LOG_TYPE;
}  // namespace ublox
#endif
