DROP SCHEMA public CASCADE;
CREATE SCHEMA public;
GRANT ALL ON SCHEMA public TO postgres;
GRANT ALL ON SCHEMA public TO public;

CREATE TABLE entities
(
    entity_id SERIAL NOT NULL,
    PRIMARY KEY (entity_id)
);

CREATE TYPE attribute_type as ENUM ('bool', 'int', 'id', 'string', 'float');

CREATE TABLE attributes
(
    attribute_name varchar(24)    NOT NULL,
    type           attribute_type NOT NULL,
    PRIMARY KEY (attribute_name)
);

/******************* ENTITY ATTRIBUTES */

CREATE TABLE entity_attributes_id
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value int         NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (attribute_value)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_str
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value varchar(24) NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE/*,
    CONSTRAINT CHK_name_unique CHECK (NumSameNames() = 0)*/
);



CREATE TABLE entity_attributes_float
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value float       NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_bool
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value bool,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/******************* FUNCTIONS */

/* Counts the number of entities that have the same name */
CREATE FUNCTION NumSameNames()
    RETURNS BIGINT
    IMMUTABLE
    LANGUAGE SQL
AS
$$
SELECT COUNT(*)
FROM entity_attributes_str
WHERE attribute_name = 'name'
  AND attribute_value = 'robot'
$$;

CREATE PROCEDURE get_concepts(INT)
    LANGUAGE SQL
AS
$$
WITH RECURSIVE cteConcepts (ID)
                   AS
                   (
                       SELECT attribute_value
                       FROM entity_attributes_id
                       WHERE attribute_name = "instance_of"
                         AND entity_id = $1

                       UNION ALL

                       SELECT a.attribute_value
                       FROM entity_attributes_id a
                                INNER JOIN cteConcepts b
                                           ON a.attribute_name = "is_a"
                                               AND a.entity_id = b.ID
                   )
SELECT ID
FROM cteConcepts;
$$;

/***** DEFAULT VALUES */

insert into attributes(attribute_name, type)
values ('answer_to', 'int');
insert into attributes(attribute_name, type)
values ('default_location', 'int');
insert into attributes(attribute_name, type)
values ('has', 'int');
insert into attributes(attribute_name, type)
values ('height', 'float');
insert into attributes(attribute_name, type)
values ('instance_of', 'int');
insert into attributes(attribute_name, type)
values ('is_a', 'int');
insert into attributes(attribute_name, type)
values ('is_concept', 'bool');
insert into attributes(attribute_name, type)
values ('is_connected', 'int');
insert into attributes(attribute_name, type)
values ('is_delivered', 'int');
insert into attributes(attribute_name, type)
values ('is_facing', 'int');
insert into attributes(attribute_name, type)
values ('is_holding', 'int');
insert into attributes(attribute_name, type)
values ('is_in', 'int');
insert into attributes(attribute_name, type)
values ('is_near', 'int');
insert into attributes(attribute_name, type)
values ('is_open', 'bool');
insert into attributes(attribute_name, type)
values ('is_placed', 'int');
insert into attributes(attribute_name, type)
values ('name', 'string');


insert into entities(entity_id)
values (1);
insert into entities(entity_id)
values (2);
insert into entity_attributes_str(entity_id, attribute_name, attribute_value)
values (2, 'name', 'robot');
insert into entity_attributes_bool(entity_id, attribute_name, attribute_value)
values (2, 'is_concept', TRUE);
insert into entity_attributes_id(entity_id, attribute_name, attribute_value)
values (1, 'instance_of', 2);
