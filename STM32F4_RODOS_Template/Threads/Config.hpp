#ifndef FLOATSAT_THREADS_CONFIG_HPP_
#define FLOATSAT_THREADS_CONFIG_HPP_


/**
 * @todo:   - Implement stepper config settings
 *          - Change the number of steps per mm to the correct value.
 *          
*/

namespace config
{
    // WARNING: not fully done / implemented yet

    /* ----------------------------------------- Threads -----------------------------------------  */
    // Control
    inline const bool control_thread_enable = true;
    inline const int  control_thread_period = 500;     // [ms]

    // ELetrical Monitoring. (DONT MESS WITH THIS! OR BEWARE OF THE MAGIC SMOKE/EXPLOSIONS!)
    inline const bool electrical_monitoring_thread_enable = true; //DO NOT DISABLE THIS THREAD. IT IS NEEDED TO PROTECT THE BATTERY! ONLY DISABLE IF YOU KNOW WHAT YOU ARE DOING AND REALLY NEED TO!
    inline const int  electrical_monitoring_thread_period = 100;       // [ms]

    /* ---------------------------------------- Hardware ---------------------------------------  */

    // HBridge
    inline const int pwmFrequency = 2000;       // [Hz]
    inline const int pwmIncrements = 500;

    // Raspberry Pi
    inline const bool enable_rpi = false;      // Will not turn on rpi if true


    /* ------------------------------------------ Com -----------------------------------------  */

}

#endif