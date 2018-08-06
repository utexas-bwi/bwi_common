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

/******************* ENTITY ATTRIBUTES */

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
        ON UPDATE CASCADE,
    CONSTRAINT CHK_name_unique CHECK (NumSameNames() = 0)
);

/******************* FUNCTIONS */
DELIMITER //
/* Counts the number of entities that have the same name */
CREATE FUNCTION NumSameNames()
RETURNS int
DETERMINISTIC
BEGIN
    DECLARE name_same_count INT unsigned DEFAULT 0;
    SELECT count(*)
    FROM entity_attributes_str l
    INNER JOIN entity_attributes_str r
    ON r.attribute_name = "name"
        AND l.attribute_value = r.attribute_value
    INTO name_same_count;
    RETURN name_same_count;
END//

DELIMITER ;

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
insert into attributes(attribute_name) values('default_location');
insert into attributes(attribute_name) values('has');
insert into attributes(attribute_name) values('instance_of');
insert into attributes(attribute_name) values('is_a');
insert into attributes(attribute_name) values('is_concept');
insert into attributes(attribute_name) values('is_connected');
insert into attributes(attribute_name) values('is_delivered');
insert into attributes(attribute_name) values('is_facing');
insert into attributes(attribute_name) values('is_holding');
insert into attributes(attribute_name) values('is_in');
insert into attributes(attribute_name) values('is_located');
insert into attributes(attribute_name) values('is_near');
insert into attributes(attribute_name) values('is_placed');
insert into attributes(attribute_name) values('name');
insert into attributes(attribute_name) values('question');
insert into attributes(attribute_name) values('sensed');

/***** DEFAULT VALUES */

insert into entities(entity_id) values(1);
insert into entities(entity_id) values(2);
insert into entity_attributes_str(entity_id, attribute_name, attribute_value) values(2, 'name', 'robot');
insert into entity_attributes_bool(entity_id, attribute_name, attribute_value) values(2, 'is_concept', TRUE);
insert into entity_attributes_id(entity_id, attribute_name, attribute_value) values(1, 'instance_of', 2);