"""OpenAPI のスキーマ部分集合（type / nullable / required / properties / additionalProperties / items / enum /
minimum / maximum / minLength / maxLength / $ref）の検証。外部ライブラリなし。違反は文字列のリストで返す。"""

_TYPES = {"object": dict, "array": list, "string": str, "boolean": bool, "number": (int, float), "integer": int}


def check(value, schema, components, path="$"):
    if "$ref" in schema:
        return check(value, components[schema["$ref"].rsplit("/", 1)[1]], components, path)
    if value is None:
        return [] if schema.get("nullable") else ["%s: null は許可されていない" % path]
    want = schema.get("type")
    if want:
        ok = isinstance(value, _TYPES[want]) and not (want in ("integer", "number") and isinstance(value, bool)) \
            and not (want == "integer" and isinstance(value, float))
        if not ok:
            return ["%s: %s のはずが %s" % (path, want, type(value).__name__)]
    errs = []
    if "enum" in schema and value not in schema["enum"]:
        errs.append("%s: %r は enum %r にない" % (path, value, schema["enum"]))
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if "minimum" in schema and value < schema["minimum"]:
            errs.append("%s: %r < minimum" % (path, value))
        if "maximum" in schema and value > schema["maximum"]:
            errs.append("%s: %r > maximum" % (path, value))
    if isinstance(value, str):
        if "minLength" in schema and len(value) < schema["minLength"]:
            errs.append("%s: 短すぎる" % path)
        if "maxLength" in schema and len(value) > schema["maxLength"]:
            errs.append("%s: 長すぎる" % path)
    if isinstance(value, list) and "items" in schema:
        for i, x in enumerate(value):
            errs += check(x, schema["items"], components, "%s[%d]" % (path, i))
    if isinstance(value, dict):
        props, extra = schema.get("properties", {}), schema.get("additionalProperties", True)
        errs += ["%s: 必須の %r がない" % (path, r) for r in schema.get("required", []) if r not in value]
        for k, x in value.items():
            if k in props:
                errs += check(x, props[k], components, "%s.%s" % (path, k))
            elif extra is False:
                errs.append("%s: 定義にないキー %r" % (path, k))
            elif isinstance(extra, dict):
                errs += check(x, extra, components, "%s.%s" % (path, k))
    return errs
