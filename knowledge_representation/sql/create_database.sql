drop database if exists villa_krr;
create database villa_krr;
use villa_krr;

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

insert into attributes(attribute_name) values('concept');
insert into attributes(attribute_name) values('is_a');
insert into attributes(attribute_name) values('is_in');
insert into attributes(attribute_name) values('is_facing');
insert into attributes(attribute_name) values('is_located');
insert into attributes(attribute_name) values('is_placed');
insert into attributes(attribute_name) values('is_delivered');
insert into attributes(attribute_name) values('is_holding');
insert into attributes(attribute_name) values('default_location');
insert into attributes(attribute_name) values('sensed');
insert into attributes(attribute_name) values('person_name');
insert into attributes(attribute_name) values('map_name');
insert into attributes(attribute_name) values('question');
insert into attributes(attribute_name) values('answer_to');

/***** DEFAULT VALUES */

insert into entities(entity_id) values(1);
insert into entities(entity_id) values(2);
insert into entity_attributes_str(entity_id, attribute_name, attribute_value) values(2, 'concept', 'robot');
insert into entity_attributes_id(entity_id, attribute_name, attribute_value) values(1, 'is_a', 2);


/*
How many chairs are in the livingroom?

First - Select everything that's in something.
select * from entity_attributes where attribute_name='is_in'

Second - Select the concept of a livingroom.
select * from entity_attributes where attribute_name='concept' and attribute_value_string='livingroom'

Third - Select the entities that are livingrooms.
select entity_id from (select t1.entity_id from entity_attributes t1, entity_attributes t2 where t1.attribute_value_entity_id = t2.entity_id and t1.attribute_name='is_a' and t2.attribute_name='concept' and t2.attribute_value_string='livingroom') t3;

Third - Select everything that's a chair.
select entity_id from (select t1.entity_id from entity_attributes t1, entity_attributes t2 where t1.attribute_value_entity_id = t2.entity_id and t1.attribute_name='is_a' and t2.attribute_name='concept' and t2.attribute_value_string='chair') t4;

Fourth - Select every chair's is_in attribute;
select * from entity_attributes t5 inner join t5.entity_id in (select entity_id from (select t1.entity_id from entity_attributes t1, entity_attributes t2 where t1.attribute_value_entity_id = t2.entity_id and t1.attribute_name='is_a' and t2.attribute_name='concept' and t2.attribute_value_string='chair') t4);
*/

/*
Where is the coke?

select attribute_value_string from entity_attributes where attribute_name='concept' and entity_id=(select attribute_value_entity_id from entity_attributes where attribute_name='is_a' and entity_id=(select attribute_value_entity_id from entity_attributes where attribute_name='is_on' and entity_id=(select entity_id from entity_attributes where attribute_name='is_a' and attribute_value_entity_id=(select entity_id from entity_attributes where attribute_value_string='coke'))));

This gets you the type of the thing that the coke is on.
select attribute_value_entity_id from entity_attributes where attribute_name='is_a' and entity_id=(select attribute_value_entity_id from entity_attributes where attribute_name='is_on' and entity_id=(select entity_id from entity_attributes where attribute_name='is_a' and attribute_value_entity_id=(select entity_id from entity_attributes where attribute_value_string='coke')));

This gets you the thing that the coke is on.
select attribute_value_entity_id from entity_attributes where attribute_name='is_on' and entity_id=(select entity_id from entity_attributes where attribute_name='is_a' and attribute_value_entity_id=(select entity_id from entity_attributes where attribute_value_string='coke'));

This gets you the concrete 'coke' entity.
select entity_id from entity_attributes where attribute_name='is_a' and attribute_value_entity_id=(select entity_id from entity_attributes where attribute_value_string='coke');

insert into entities(entity_id) values(3);
insert into entity_attributes(entity_id, attribute_name, attribute_value_entity_id) values(3, 'is_a', 0);
*/
