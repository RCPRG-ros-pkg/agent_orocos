SubContainer cont1      # subsystem_buffer{type: container; validity: cont1_valid}
bool cont1_valid

SubContainer cont2      # subsystem_buffer{type: container}

SubContainer cont3      # subsystem_buffer{type: port; data_type: SubContainer; validity: cont3_valid}
bool cont3_valid

SubContainer cont4      # subsystem_buffer{type: port; data_type: SubContainer}

uint32 field1           # subsystem_buffer{type: port; data_type: uint32_t; validity: field1_valid}
bool field1_valid

uint32 field2           # subsystem_buffer{type: port; data_type: uint32_t}

