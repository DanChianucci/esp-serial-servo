WaveshareST3215 Smart Servo
Robotshop



TODO:
  Add Arduino Wire Backend Support

  Remove c++20 dependencies
     std::span
     std::expected
     initializer lists??

  Split Protocol from memory map more cleanly


  serial_servo
    protocol
      base_protocol.h    -> abstract impl of some functions
      dynamixel_one.h    ->
      stseries.h         -> little endian
      scseries.h         -> bigendian
      dynamixel_two.h    -> 4 byte sync, extra response byte
    servos
      st3215.h
      sc15.h



    template<SYNC_HEADER, ENDIANNESS, ... >
    class base_protocol{...}

    class STSeriesPacket : base_protocol<>{...}
                  or
    typedef base_protocol<> STSeriesPacket;





  //DynamixelOne
  //
  //



  //Waveshare STS Series: https://www.waveshare.com/wiki/ST3215_Servo#Document
  //  Waveshare ST3020
  //  Waveshare ST3025
  //  Waveshare ST3215
  //  Waveshare ST3235


  //Waveshare SCS Series : https://www.waveshare.com/wiki/SC09_Servo#Document
  // Looks to be the same as dynamixel protocol, but big endian
  //  Waveshare SC09
  //  Waveshare SC15
  //  Waveshare CF35-12


  //Hiwonder
  //  Same protocol as dynamixel, different sync header
  //  HiWonder  LX-15D
  //            LX-16A
  //            LX-224,
  //            HTS-35H and
  //            ...


  //Dynamixel2.0
  //  Same as Dynamixel 1.0 w/
  //       larger syncheader      FF_FF_FD_00
  //       length field is        2 bytes
  //       response contains inst and error instead of just error








  Add LXSeries Servo Support
     - similiar protocol to STSeries

  Add SCSeries Servo Support
     - Same Protocol as STSeries byte endianness is swapped
     - different memory map

  Add Dynamixel 1.0 Support
     - Same Protocol as STSeries
     -different memory map per servo

  Add Dynamixel 2.0
    -Similiar to STSeries
      - longer sync header
      - response has instruction + status bytes
