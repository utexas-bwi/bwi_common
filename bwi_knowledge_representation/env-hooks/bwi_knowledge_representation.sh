#!/usr/bin/env sh

prepare_knowledge() {
    knowledge_rep_path=$(rospack find knowledge_representation) &&
    # Drop tables and establish schema
    mysql -u root -p -e "source $knowledge_rep_path/sql/create_database.sql" &&

    # Load arena prespecified knowledge
    yaml_files_path=$(rospack find utexas_gdc)/maps/real/3 &&
    rosrun knowledge_representation populate_with_map $yaml_files_path

    #Load additional knowledge
    knowledge_file_path=$(rospack find utexas_gdc)/knowledge/bwi.yaml &&
    rosrun knowledge_representation populate_with_knowledge $knowledge_file_path

}