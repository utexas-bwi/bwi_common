drop database if exists knowledge_base;
create database knowledge_base;
use knowledge_base;

CREATE TABLE entities (
    entity_id int NOT NULL AUTO_INCREMENT,
    PRIMARY KEY(entity_id)
);

CREATE TABLE attributes (
    attribute_name varchar(24) NOT NULL,
    PRIMARY KEY(attribute_name)
);

CREATE TABLE entity_attributes_id (
    entity_id int NOT NULL,
    attribute_name varchar(24) NOT NULL,
    attribute_value int NOT NULL,
    PRIMARY KEY(entity_id, attribute_name, attribute_value),
    FOREIGN KEY(attribute_value)
        REFERENCES entities(entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY(entity_id)
        REFERENCES entities(entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY(attribute_name)
        REFERENCES attributes(attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_str (
    entity_id int NOT NULL,
    attribute_name varchar(24) NOT NULL,
    attribute_value varchar(24) NOT NULL,
    PRIMARY KEY(entity_id, attribute_name, attribute_value),
    FOREIGN KEY(entity_id)
        REFERENCES entities(entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY(attribute_name)
        REFERENCES attributes(attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_float (
    entity_id int NOT NULL,
    attribute_name varchar(24) NOT NULL,
    attribute_value float NOT NULL,
    PRIMARY KEY(entity_id, attribute_name, attribute_value),
    FOREIGN KEY(entity_id)
        REFERENCES entities(entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY(attribute_name)
        REFERENCES attributes(attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_bool (
    entity_id int NOT NULL,
    attribute_name varchar(24) NOT NULL,
    attribute_value bool,
    PRIMARY KEY(entity_id, attribute_name, attribute_value),
    FOREIGN KEY(entity_id)
        REFERENCES entities(entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY(attribute_name)
        REFERENCES attributes(attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

insert into attributes(attribute_name) values('answer_to');
insert into attributes(attribute_name) values('concept');
insert into attributes(attribute_name) values('default_location');
insert into attributes(attribute_name) values('has_door');
insert into attributes(attribute_name) values('is_a');
insert into attributes(attribute_name) values('is_connected');
insert into attributes(attribute_name) values('is_delivered');
insert into attributes(attribute_name) values('is_facing');
insert into attributes(attribute_name) values('is_holding');
insert into attributes(attribute_name) values('is_in');
insert into attributes(attribute_name) values('is_located');
insert into attributes(attribute_name) values('is_near');
insert into attributes(attribute_name) values('is_placed');
insert into attributes(attribute_name) values('map_name');
insert into attributes(attribute_name) values('person_name');
insert into attributes(attribute_name) values('question');
insert into attributes(attribute_name) values('sensed');

/***** DEFAULT VALUES */

insert into entities(entity_id) values(1);
insert into entities(entity_id) values(2);
insert into entity_attributes_str(entity_id, attribute_name, attribute_value) values(2, 'concept', 'robot');
insert into entity_attributes_id(entity_id, attribute_name, attribute_value) values(1, 'is_a', 2);