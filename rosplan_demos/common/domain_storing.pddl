(define (domain storing-groceries)

    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)
    (:types
        waypoint
        item
        robot
        door
        plane
    )

    (:predicates
        (waypoint ?waypoint - waypoint)
        (robot ?robot - robot)
        (robot_at ?robot - robot ?waypoint - waypoint)
        (door ?door - door)
        (door_open ?door - door)
        (plane ?plane - plane)
        (item ?item - item)
        (perceive ?plane - plane ?waypoint - waypoint)
        (perceived ?plane - plane ?waypoint - waypoint)
        (belongs_to ?plane - plane ?waypoint - waypoint)
        (on_plane ?item - item ?plane - plane)
        (store_item ?item - item)
        (empty_gripper ?robot - robot)
        (holding ?robot - robot ?item - item)
        (storeGroceries ?robot - robot ?waypointFrom - waypoint ?waypointTo - waypoint)
    )

    (:durative-action move_base
        :parameters (?robot - robot ?from ?to - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (at start (waypoint ?to))
            (at start (waypoint ?from))
            (at start (robot_at ?robot ?from))
        )
        :effect (and
            (at start (not (robot_at ?robot ?from)))
            (at end (robot_at ?robot ?to))
        )
    )

    (:durative-action open_door
        :parameters (?robot - robot ?door - door ?waypoint - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (at start (robot_at ?robot ?waypoint))
            (at start (door ?door))
            (at start (not (door_open ?door)))
        )
        :effect (and
            (at end (door_open ?door))
        )
    )

    (:durative-action perceive_plane
        :parameters (?robot - robot ?plane - plane ?waypoint - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (at start (robot_at ?robot ?waypoint))
            (at start (belongs_to ?plane ?waypoint))
            (at start (perceive ?plane ?waypoint))
        )
        :effect (and
            (at start (not (perceive ?plane ?waypoint)))
            (at end (perceived ?plane ?waypoint))
        )
    )

    (:durative-action pick
        :parameters (?robot - robot ?item - item ?plane - plane ?waypoint - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (at start (robot_at ?robot ?waypoint))
            (at start (plane ?plane))
            (at start (perceived ?plane ?waypoint))
     			  (at start (belongs_to ?plane ?waypoint))
     			  (at start (on_plane ?item ?plane))
     			  (at start (store_item ?item))
     			  (at start (empty_gripper ?robot))
        )
        :effect (and
            (at start (not (on_plane ?item ?plane)))
            (at start (not (store_item ?item)))
            (at start (not (empty_gripper ?robot)))
            (at end (holding ?robot ?item))
        )
    )
    (:durative-action place
        :parameters (?robot - robot ?item - item ?plane - plane ?waypoint - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (at start (robot_at ?robot ?waypoint))
            (at start (perceived ?plane ?waypoint))
            (at start (belongs_to ?plane ?waypoint))
            (at start (holding ?robot ?item))
        )
        :effect (and
            (at start (not (holding ?robot ?item)))
            (at start (empty_gripper ?robot))
            (at end (on_plane ?item ?plane))
        )
    )

)
