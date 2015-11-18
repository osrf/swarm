# Install Gnuplot scripts.
install (PROGRAMS swarm_comms_datarate.gplot DESTINATION ${BIN_INSTALL_DIR})
install (PROGRAMS swarm_comms_drops.gplot DESTINATION ${BIN_INSTALL_DIR})
install (PROGRAMS swarm_comms_msgs_sent.gplot DESTINATION ${BIN_INSTALL_DIR})
install (PROGRAMS swarm_comms_neighbors.gplot DESTINATION ${BIN_INSTALL_DIR})

# Install the script for generating reports.
install (PROGRAMS swarm_generate_report DESTINATION ${BIN_INSTALL_DIR})

