#!/usr/bin/env sh

save_knowledge() {
    stamp=$(date +%Y-%m-%d_%H-%M-%S)
    if [[ $# == 1 ]]; then
       stamp=$1
    fi
    mysqldump -u root -p --databases knowledge_base > knowledge_$(stamp).sql
}
