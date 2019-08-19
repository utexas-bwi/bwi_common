from actasp_ros.msg import AspFluent, AspRule


def to_fluent(name, parameters):
    formatted_params = []
    for param in parameters:

        if isinstance(param, str):
            formatted_params.append("\"" + param + "\"")
        else:
            formatted_params.append(str(param))

    fluent = AspFluent()
    fluent.name = name
    fluent.variables = formatted_params
    return fluent


def to_asp_rule_head(name, parameters):
    rule = AspRule()
    rule.head = [to_fluent(name, parameters)]
    return rule


def to_asp_rule_body(name, parameters):
    rule = AspRule()
    rule.body = [to_fluent(name, parameters)]
    return rule
