(define (problem task)
(:domain storing-groceries)
(:objects
    cupboard_1 table_1 unknown - waypoint
    bottle cereal_box - item
    lucy - robot
    cupboarddoor_1 - door
    shelf1_1 shelf1_2 table1_1 - plane
)
(:init
    (waypoint table_1)
    (waypoint cupboard_1)
    (waypoint unknown)
    (robot_at lucy unknown)
    (door cupboarddoor_1)
    (plane table1_1)
    (plane shelf1_1)
    (item bottle)
    (perceive table1_1 table_1)
    (perceive shelf1_1 cupboard_1)
    (belongs_to table1_1 table_1)
    (belongs_to shelf1_1 cupboard_1)
    (on_plane bottle table1_1)
    (store_item bottle)
    (empty_gripper lucy)
)
(:goal (and
    (on_plane bottle shelf1_1)
))
)
