#include "../bindings.h"
#include "cmission_example.h"

#include "stdio.h"

enum CMissionResult foo(struct MissionMapPtr *data) {
    printf("Hello world!\n");
    return Ok;
}


CMissionPtr* cmission_example_create() {
    struct OptionFunction_CTaskFunc foo_option;
    foo_option.tag = Some_CTaskFunc;
    foo_option.some = &foo;

    struct OptionFunction_CTaskFunc null_option;
    null_option.tag = None_CTaskFunc;
    null_option.some = NULL;

    struct CTask *task = ctask_create("foo", foo_option, null_option);
    CTask *task_array[1] = {task};

    struct CMissionPtr *mission = cmission_create("mission_example", *task_array, 1);

    return mission;
}