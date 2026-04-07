#include <pubsub.h>
#include <stdlib.h>

typedef struct function_list {
    subscriber_callback_t callback;
    struct function_list *next;
} function_list_t;

static function_list_t *g_function_pointers[TOPIC_NULL];

void publish(topic_t topic, uint8_t *data, size_t size) {
    function_list_t *cur = g_function_pointers[topic];
    while (cur != NULL) {
        if (cur->callback != NULL) {
            cur->callback(data, size);
        }
        cur = cur->next;
    }
}

void subscribe(topic_t topic, subscriber_callback_t callback) {
    function_list_t *fn = (function_list_t *)malloc(sizeof(function_list_t));
    fn->callback = callback;
    fn->next = NULL;

    if (g_function_pointers[topic] == NULL) {
        g_function_pointers[topic] = fn;
    } else {
        function_list_t *cur = g_function_pointers[topic];
        while (cur->next != NULL) {
            cur = cur->next;
        }
        cur->next = fn;
    }
}
