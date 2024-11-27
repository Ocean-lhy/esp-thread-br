idf_component_register(SRCS "esp_ot_br.cpp"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem
                       )
