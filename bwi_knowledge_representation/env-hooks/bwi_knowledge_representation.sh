#!/usr/bin/env sh

prepare_knowledge_bwi() {
    knowledge_rep_path=$(rospack find knowledge_representation) &&
    # Drop tables and establish schema
    #mysql -u root -p -e "source $knowledge_rep_path/sql/schema_mysql.sql" &&
    sudo -u postgres psql -d knowledge_base -f $knowledge_rep_path/sql/schema_postgresql.sql

    # Load arena prespecified knowledge
    yaml_files_path=$(rospack find utexas_gdc)/maps/real/3 &&
    rosrun bwi_knowledge_representation populate_with_map $yaml_files_path

    #Load additional knowledge
    knowledge_file_path=$(rospack find utexas_gdc)/knowledge/bwi.yaml &&
    rosrun knowledge_representation populate_with_knowledge $knowledge_file_path

}

prepare_knowledge_bwi_ahg() {
    knowledge_rep_path=$(rospack find knowledge_representation) &&
    # Drop tables and establish schema
    #mysql -u root -p -e "source $knowledge_rep_path/sql/schema_mysql.sql" &&
    sudo -u postgres psql -d knowledge_base -f $knowledge_rep_path/sql/schema_postgresql.sql

    # Load arena prespecified knowledge
    yaml_files_path=$(rospack find utexas_ahg)/maps/2s/occ &&
    rosrun bwi_knowledge_representation populate_with_map $yaml_files_path
    
    yaml_files_path=$(rospack find utexas_ahg)/maps/1s/occ &&
    rosrun bwi_knowledge_representation populate_with_map $yaml_files_path


    #Load additional knowledge
    knowledge_file_path=$(rospack find utexas_ahg)/knowledge/bwi.yaml &&
    rosrun knowledge_representation populate_with_knowledge $knowledge_file_path

}
