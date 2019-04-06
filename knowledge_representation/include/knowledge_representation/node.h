#pragma once
#include <string>

namespace knowledge_rep {

class Node {
public:
  Node(){}
  virtual ~Node() {}
};

class Query : public Node {
};

class Valuable : public Node {
};

class Idable : public Node {
};

class FreeVar : public Node {
  FreeVar(){}
};

class VId : public Valuable {
public:
  VId(){}
  VId(int value) : value(value) {}

  int value{};
};

class VFloat : public Valuable {
public:
  VFloat(){}
  VFloat(float value) : value(value) {}

  float value;
};

class VBool : public Valuable {
public:
  VBool(){}
  VBool(bool value) : value(value) {}

  bool value;
};

class VString : public Valuable {
public:
  VString(){}
  VString(const std::string &value) : value(value) {}

  std::string value;
};

class AttributeName : public Node {
public:
  AttributeName(){}
  AttributeName(const std::string &name) : name(name) {}

  AttributeName& build(){
    return *new AttributeName();
  }

  std::string name;
};

class QueryId : public Query, public Idable {
public:
  QueryId(){}
  FreeVar *var;
  AttributeName *name;
  Valuable *value;
};

class QueryValue : public Query, public Valuable {
public:
  QueryValue(){}
  //VId id;
  //AttributeName name;
  //FreeVar var;
};

}
