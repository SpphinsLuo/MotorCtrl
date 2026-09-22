



        void elmo_read_pos(Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_)
        {
            uint8_t positionObj = 0x63;
            std::array<uint8_t, 8> data_buf = {0x43, positionObj, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            ssize_t retSend = 0;
            do
            {
                retSend = serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            // std::cout << "retSend's value is: "<< retSend << std::endl;

            } while (!(retSend==30));

            // 为了避免读到缓存, 需要对读到的CAN帧进一步过滤
            bool res = false;
            auto start = std::chrono::high_resolution_clock::now();
            auto end = start;
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            do
            {   
                // std::cout << "the start receive_canopen_sdo_timeout"<< retSend << std::endl;

                serial_->recv((uint8_t *)&return_canObj_, 0xAA, sizeof(CAN_Receive_Frame));
                // std::cout << "the end receive_canopen_sdo_timeout"<< retSend << std::endl;
                end = std::chrono::high_resolution_clock::now();
                duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();   
            } while (!(
                        (((return_canObj_.canId & 0xFF0)==0x580) && (return_canObj_.canData[1]==positionObj))  ||   (duration>=500)
                        ));
            if (duration>=500)
            {
                std::cout << "read pos SDO time out " << std::endl;
                receive_flag_ = 0;
            }

            if (return_canObj_.CMD == 0x11 && return_canObj_.frameEnd == 0x55) // receive success
            {
                receive_flag = 1; //success
            }
            else if (return_canObj_.CMD == 0x01) // receive fail
            {
                std::cout << "receive fail" << std::endl;
                receive_flag = 0; 
            }
            else if (return_canObj_.CMD == 0x02) // send fail
            {
                /* code */
                std::cout << "send fail" << std::endl;
                receive_flag = 0; 

            }
            else if (return_canObj_.CMD == 0x03) // send success
            {
                /* code */
                std::cout << "send success" << std::endl;
                receive_flag = 0; 

            }
            else if (return_canObj_.CMD == 0xEE) // communication error
            {
                /* code */
                receive_flag = 0; 
                std::cout << "communication error" << std::endl;

            }
        }
            