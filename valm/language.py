import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LanguageNode(Node):

    def __init__(self):
        super().__init__("language")

        self.scene = {
            "objects": []
        }

        self.create_subscription(
            String,
            "/scene_state",
            self.scene_callback,
            10
        )

        self.create_subscription(
            String,
            "/user_instruction",
            self.instruction_callback,
            10
        )

        self.plan_publisher = self.create_publisher(
            String,
            "/action_plan",
            10
        )

        self.get_logger().info(
            "Generic language planner started"
        )

    def scene_callback(self, msg):
        try:
            scene = json.loads(msg.data)

            if not isinstance(scene, dict):
                raise ValueError(
                    "Scene must be a JSON object"
                )

            if "objects" not in scene:
                raise ValueError(
                    "Scene does not contain an 'objects' field"
                )

            if not isinstance(scene["objects"], list):
                raise ValueError(
                    "'objects' must be a list"
                )

            self.scene = scene

            self.get_logger().info(
                f"Scene updated with "
                f"{len(self.scene['objects'])} objects"
            )
            
            # For testing
            for obj in self.scene["objects"]:

                object_id = obj.get("id")
                label = obj.get("label")
                confidence = obj.get("confidence")
                position = obj.get("position")

                self.get_logger().info(
                    f"{object_id}: "
                    f"{label}, "
                    f"conf={confidence:.2f}, "
                    f"position={position}"
                )

        except (json.JSONDecodeError, ValueError) as error:
            self.get_logger().error(
                f"Invalid scene state: {error}"
            )

    def instruction_callback(self, msg):
        instruction = msg.data.strip()

        if not instruction:
            self.get_logger().warning(
                "Received empty instruction"
            )
            return

        self.get_logger().info(
            f"Instruction received: {instruction}"
        )

        if not self.scene["objects"]:
            self.get_logger().warning(
                "No objects currently available in scene"
            )

        plan = self.generate_plan(
            instruction,
            self.scene
        )

        if not self.validate_plan(plan):
            self.get_logger().error(
                "Generated plan failed validation"
            )
            return

        output = String()
        output.data = json.dumps(plan)

        self.plan_publisher.publish(output)

        self.get_logger().info(
            f"Published action plan: {output.data}"
        )

    def generate_plan(self, instruction, scene):
        prompt = self.build_prompt(
            instruction,
            scene
        )

        self.get_logger().info(
            f"\nGenerated planner prompt:\n{prompt}"
        )

        
        # LLM goes here

        return {
            "instruction": instruction,
            "actions": []
        }

    def build_prompt(self, instruction, scene):
        objects = scene.get(
            "objects",
            []
        )

        object_lines = []

        for obj in objects:
            object_id = obj.get(
                "id",
                "unknown"
            )

            label = obj.get(
                "label",
                "unknown"
            )

            confidence = obj.get(
                "confidence"
            )

            position = obj.get(
                "position"
            )

            line = (
                f"- id: {object_id}, "
                f"label: {label}"
            )

            if confidence is not None:
                line += (
                    f", confidence: "
                    f"{confidence:.2f}"
                )

            if position is not None:
                line += (
                    f", position: "
                    f"{position}"
                )

            object_lines.append(line)

        if object_lines:
            object_description = "\n".join(
                object_lines
            )

        else:
            object_description = (
                "- No objects detected"
            )

        return f"""
You are the high-level task planner for a robot.

Your job is to convert the user's natural-language
instruction into a sequence of valid robot actions.

You do NOT control motors, joints, trajectories,
or low-level robot hardware.

You only produce high-level actions.


CURRENT SCENE

{object_description}


AVAILABLE ACTIONS

1. pick(object_id)

Pick up an object that exists in the current scene.

JSON example:

{{
    "action": "pick",
    "object_id": "object_0"
}}


2. place(object_id, target)

Place an object at a named target.

JSON example:

{{
    "action": "place",
    "object_id": "object_0",
    "target": "assembly_area"
}}


3. move_relative(x, y, z)

Move the robot end effector relative to its
current position.

x, y and z are distances in metres.

Positive and negative values represent movement
along the robot coordinate axes.

The robot execution system determines the exact
coordinate frame.

Example: move 5 cm in positive X:

{{
    "action": "move_relative",
    "x": 0.05,
    "y": 0.0,
    "z": 0.0
}}

Example: move 3 cm upward:

{{
    "action": "move_relative",
    "x": 0.0,
    "y": 0.0,
    "z": 0.03
}}


4. stop()

Stop execution.

JSON example:

{{
    "action": "stop"
}}


RULES

1. Only use object IDs that appear in CURRENT SCENE.

2. Never invent an object ID.

3. Do not generate joint angles.

4. Do not generate motor commands.

5. Do not generate trajectories.

6. Do not generate absolute robot coordinates.

7. Only use the actions listed above.

8. All move_relative values must be in metres.

9. Each individual relative movement must be
   no greater than 0.20 metres on any axis.

10. If an instruction cannot be completed with
    the currently available objects or actions,
    return an error instead of inventing information.

11. Return ONLY valid JSON.

12. Do not include markdown.

13. Do not explain the plan.


REQUIRED OUTPUT FORMAT

{{
    "actions": [
        {{
            "action": "pick",
            "object_id": "object_0"
        }},
        {{
            "action": "move_relative",
            "x": 0.0,
            "y": 0.0,
            "z": 0.05
        }},
        {{
            "action": "place",
            "object_id": "object_0",
            "target": "assembly_area"
        }}
    ]
}}


IF THE TASK CANNOT BE COMPLETED

{{
    "actions": [],
    "error": "reason the task cannot be completed"
}}


USER INSTRUCTION

{instruction}
""".strip()

    def validate_plan(self, plan):
        if not isinstance(
            plan,
            dict
        ):
            self.get_logger().error(
                "Plan must be a dictionary"
            )
            return False

        actions = plan.get(
            "actions"
        )

        if not isinstance(
            actions,
            list
        ):
            self.get_logger().error(
                "Plan must contain an actions list"
            )
            return False

        valid_object_ids = {
            obj.get("id")
            for obj in self.scene.get(
                "objects",
                []
            )
            if obj.get("id") is not None
        }

        allowed_actions = {
            "pick",
            "place",
            "move_relative",
            "stop"
        }

        max_relative_move = 0.20

        for index, action in enumerate(actions):
            if not isinstance(
                action,
                dict
            ):
                self.get_logger().error(
                    f"Action {index} is not "
                    f"a dictionary"
                )
                return False

            action_name = action.get(
                "action"
            )

            if action_name not in allowed_actions:
                self.get_logger().error(
                    f"Unknown action: "
                    f"{action_name}"
                )
                return False

          
            # Pick
            

            if action_name == "pick":
                object_id = action.get(
                    "object_id"
                )

                if object_id is None:
                    self.get_logger().error(
                        "Pick action requires "
                        "an object_id"
                    )
                    return False

                if object_id not in valid_object_ids:
                    self.get_logger().error(
                        f"Invalid object ID "
                        f"for pick: {object_id}"
                    )
                    return False

           
            # Place
            

            elif action_name == "place":
                object_id = action.get(
                    "object_id"
                )

                target = action.get(
                    "target"
                )

                if object_id is None:
                    self.get_logger().error(
                        "Place action requires "
                        "an object_id"
                    )
                    return False

                if object_id not in valid_object_ids:
                    self.get_logger().error(
                        f"Invalid object ID "
                        f"for place: {object_id}"
                    )
                    return False

                if not isinstance(
                    target,
                    str
                ) or not target.strip():
                    self.get_logger().error(
                        "Place action requires "
                        "a valid target"
                    )
                    return False

            # Move
            
            elif action_name == "move_relative":
                for axis in [
                    "x",
                    "y",
                    "z"
                ]:
                    value = action.get(
                        axis
                    )

                    if not isinstance(
                        value,
                        (int, float)
                    ):
                        self.get_logger().error(
                            f"move_relative requires "
                            f"a numeric {axis} value"
                        )
                        return False

                    if abs(value) > max_relative_move:
                        self.get_logger().error(
                            f"Relative {axis} movement "
                            f"{value} m exceeds maximum "
                            f"{max_relative_move} m"
                        )
                        return False

            # Stop
            
            elif action_name == "stop":
                continue

        return True


def main(args=None):
    rclpy.init(
        args=args
    )

    node = LanguageNode()

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
