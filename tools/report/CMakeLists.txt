# Install the tex template for the report.
install (FILES report.tex DESTINATION ${CMAKE_INSTALL_PREFIX}/share/swarm/)

# Install Gnuplot scripts.
install (PROGRAMS comms_datarate.gplot DESTINATION ${BIN_INSTALL_DIR})
install (PROGRAMS comms_drops.gplot DESTINATION ${BIN_INSTALL_DIR})
install (PROGRAMS comms_msgs_sent.gplot DESTINATION ${BIN_INSTALL_DIR})
install (PROGRAMS comms_neighbors.gplot DESTINATION ${BIN_INSTALL_DIR})

# Install the script for generating reports.
install (PROGRAMS generate_report.sh DESTINATION ${BIN_INSTALL_DIR})

