from typing import Optional
from controller import Node

# CHATGPT WROTE THIS FUNCTION
def GetNodeByName(root_node : Node, target_name : str)-> Optional[Node]:
    # If this node has a name field, check it
    
    name_field = root_node.getField("name")
    if name_field:
        print("THERE IS A NAMEFIELD : ", name_field.getSFString())
        if name_field.getSFString() == target_name:
            return root_node

    # Search children fields
    for field_name in ["children", "endPoint"]:
        field = root_node.getField(field_name)
        if field is None:
            print("WHAAAT THERE IS NO FUCKING ", field_name)
            continue

        if field.getTypeName() == "MFNode":
            for i in range(field.getCount()):
                result = GetNodeByName(field.getMFNode(i), target_name)
                if result:
                    return result

        elif field.getTypeName() == "SFNode":
            child = field.getSFNode()
            if child:
                result = GetNodeByName(child, target_name)
                if result:
                    return result

    return None