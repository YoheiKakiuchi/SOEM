#define EC_TIMEOUTMON 500



extern int jsk_elmo_settings(int dev_no, uint8_t control_mode,
                             int cycle_timeout, int aux_position);
extern int jsk_elmo_PDO_mapping(int dev_no, uint16_t *rxpdo_list, int rxpdo_num,
                                uint16_t *txpdo_list, int txpdo_num);

// deprecated
extern int jsk_elmo_settings_old(int dev_no);
extern int jsk_elmo_PDO_mapping_old(int dev_no);
