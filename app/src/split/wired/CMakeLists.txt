target_sources(app PRIVATE wired.c)
target_sources_ifdef(CONFIG_ZMK_SPLIT_ROLE_CENTRAL app PRIVATE central.c)
target_sources_ifndef(CONFIG_ZMK_SPLIT_ROLE_CENTRAL app PRIVATE peripheral.c)