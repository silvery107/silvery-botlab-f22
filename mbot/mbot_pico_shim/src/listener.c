#include <mbot_pico_shim/listener.h>
#include <mbot_pico_shim/protocol.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdbool.h>

bool listener_running;

void* comms_listener_loop(void* data)
{
    uint8_t header_data[ROS_HEADER_LENGTH];
    header_data[0] = 0x00;
    // loop for eternity
    // TODO:
    // refactor the elements of this loop (read header, validate header, read message, validate message, etc.)
    // into seperate functions/util libraries as necessary for readability
    bool valid_header = true;
    bool valid_message = true;
    listener_running = true;
    while(listener_running)
    {
        //printf("listener running!\r\n");
        valid_header = true;
        valid_message = true;

        /*
        * Header read section
        */

        // read from serial until we get a trigger char
        char trigger_val = 0x00;
        int rc = 0x00;
        while(trigger_val != 0xff && listener_running && rc != 1)
        {
            //printf("listener running....2!\r\n");
            rc = read(*serial_device_ptr, &trigger_val,1); //stdio_usb_in_chars_itf(1, &trigger_val, 1);
        }
        //printf("listener running....4!\r\n");
        header_data[0] = trigger_val;

        valid_header = true;

        // read the rest of the header
        rc = read(*serial_device_ptr, &header_data[1], ROS_HEADER_LENGTH - 1); //stdio_usb_in_chars_itf(1, &header_data[1], ROS_HEADER_LENGTH - 1);
        valid_header = valid_header && (rc == ROS_HEADER_LENGTH - 1);

        /*
        * Header validate section
        */

        // if we received all the header bytes, check the integrity
        if(valid_header)
        {
            // check the sync flag/protocol version
            valid_header = valid_header && (header_data[1] == 0xfe);
            // compute the checksum on the received message length
            // and compare it to the received checksum
            uint8_t cs1_addends[2] = {header_data[2], header_data[3]};
            uint8_t cs_msg_len = checksum(cs1_addends, 2);
            valid_header = valid_header && (cs_msg_len == header_data[4]);
        }


        // if we get in here, then we consider our header valid
        if(valid_header)
        {

        /*
        * Message read section
        */
            uint16_t message_len = ((uint16_t)header_data[3] << 8) + (uint16_t)header_data[2];
            uint16_t topic_id = ((uint16_t)header_data[6] << 8) + (uint16_t)header_data[5];
            uint8_t msg_data_serialized[message_len];
            // TODO: refactor these kind of "read N bytes with timeout" loops into a util function
            int avail = 0;
            while(avail < (message_len + 1))
            {
                ioctl(*serial_device_ptr, FIONREAD, &avail);
            }
            //while(tud_cdc_n_available(1) < (message_len + 1))
            //{
                //spin until the full set of message bytes have arrived
            //}
            rc = read(*serial_device_ptr, msg_data_serialized, message_len);//stdio_usb_in_chars_itf(1, msg_data_serialized, message_len);
            valid_message = valid_message & (rc == message_len);
            // read in the final checksum byte
            char topic_msg_data_checksum = 0;
            rc = read(*serial_device_ptr, &topic_msg_data_checksum, 1);//stdio_usb_in_chars_itf(1, &topic_msg_data_checksum, 1);
            valid_message = valid_message && (rc == 1);

        /*
        * Message validate section
        */
            uint8_t cs2_addends[message_len + 2]; //create array for the checksum over topic and message content
            cs2_addends[0] = header_data[5];
            cs2_addends[1] = header_data[6];
            for (int i = 0; i < message_len; i++) {
                cs2_addends[i + 2] = msg_data_serialized[i];
            }

            //compute checksum over message data and topic and compare with received checksum
            uint8_t cs_topic_msg_data = checksum(cs2_addends, message_len + 2);
            valid_message = valid_message && (cs_topic_msg_data == topic_msg_data_checksum);

            // if we get in here, then both header and message were considered valid
            if(valid_message)
            {
                // see if we have a deserializer
                topic_registry_val_t topic_val;
                // the topic data structure calls calloc, so don't let us be killed in the middle of that process
                pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
                if(comms_get_topic_serializers(topic_id, &topic_val))
                {
                    //printf("set topic data for topic id %d\r\n", topic_id);
                    comms_set_topic_data(topic_id, msg_data_serialized, message_len);
                    if(topic_val.cb_fn != NULL)
                    {
                        topic_val.cb_fn(msg_data_serialized);
                    }
                }
                pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
            }
        }
        // only need to reset the first byte to restart the read loop
        header_data[0] = 0x00;
        //sleep_us(1); // brief sleep to allow FIFO to flush
    }
    return NULL;
}
