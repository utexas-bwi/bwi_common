#ifndef KNOWLEDGE_REPRESENTATION_NODE_H
#define KNOWLEDGE_REPRESENTATION_NODE_H

#include <string>

class Node {
public:
  virtual ~Node() {}
};

class Query : public Node {
};

class Valuable : public Node {
};

class Idable : public Node {
};

class FreeVar : public Node {
};

class VId : public Valuable {
public:
  VId(int value) : value(value) {}

  int value{};
};

class VFloat : public Valuable {
public:
  VFloat(float value) : value(value) {}

  float value;
};

class VBool : public Valuable {
public:
  VBool(bool value) : value(value) {}

  bool value;
};

class VString : public Valuable {
public:
  VString(const std::string &value) : value(value) {}

  std::string value;
};

class AttributeName : public Node {
public:
  AttributeName(const std::string &name) : name(name) {}

  std::string name;
};

class QueryId : public Query, public Idable {
public:
  FreeVar& var;
  AttributeName& name;
  Valuable& value;
};

class QueryValue : public Query, public Valuable {
public:
  VId id;
  AttributeName name;
  FreeVar var;
};

#endif //KNOWLEDGE_REPRESENTATION_NODE_H
