#include "allocator.h"
#include "list4mesh.h"
#include "osi.h"

struct list4mesh_node_t {
    struct list4mesh_node_t *next;
    void *data;
};

typedef struct list4mesh_t {
    list4mesh_node_t *head;
    list4mesh_node_t *tail;
    size_t length;
    list4mesh_free_cb free_cb;
    const allocator_t *allocator;
} list4mesh_t;

//static list4mesh_node_t *list4mesh_free_node_(list4mesh_t *list4mesh, list4mesh_node_t *node);

// Hidden constructor, only to be used by the hash map for the allocation tracker.
// Behaves the same as |list4mesh_new|, except you get to specify the allocator.
list4mesh_t *list4mesh_new_internal(list4mesh_free_cb callback, const allocator_t *zeroed_allocator)
{
    list4mesh_t *list4mesh = (list4mesh_t *)zeroed_allocator->alloc(sizeof(list4mesh_t));
    if (!list4mesh) {
        return NULL;
    }

    list4mesh->head = list4mesh->tail = NULL;
    list4mesh->length = 0;
    list4mesh->free_cb = callback;
    list4mesh->allocator = zeroed_allocator;
    return list4mesh;
}

list4mesh_t *list4mesh_new(list4mesh_free_cb callback)
{
    return list4mesh_new_internal(callback, &allocator_calloc);
}

void list4mesh_free(list4mesh_t *list4mesh)
{
    if (!list4mesh) {
        return;
    }

    list4mesh_clear(list4mesh);
    list4mesh->allocator->free(list4mesh);
}

bool list4mesh_is_empty(const list4mesh_t *list4mesh)
{
    assert(list4mesh != NULL);
    return (list4mesh->length == 0);
}

bool list4mesh_contains(const list4mesh_t *list4mesh, const void *data)
{
  assert(list4mesh != NULL);
  assert(data != NULL);

  for (const list4mesh_node_t *node = list4mesh_begin(list4mesh); node != list4mesh_end(list4mesh); node = list4mesh_next(node)) {
    if (list4mesh_node(node) == data)
      return true;
  }

  return false;
}

size_t list4mesh_length(const list4mesh_t *list4mesh)
{
    assert(list4mesh != NULL);
    return list4mesh->length;
}

void *list4mesh_front(const list4mesh_t *list4mesh)
{
    assert(list4mesh != NULL);
    assert(!list4mesh_is_empty(list4mesh));

    return list4mesh->head->data;
}

void *list4mesh_back(const list4mesh_t *list4mesh) {
  assert(list4mesh != NULL);
  assert(!list4mesh_is_empty(list4mesh));

  return list4mesh->tail->data;
}

list4mesh_node_t *list4mesh_back_node(const list4mesh_t *list4mesh) {
  assert(list4mesh != NULL);
  assert(!list4mesh_is_empty(list4mesh));

  return list4mesh->tail;
}

bool list4mesh_insert_after(list4mesh_t *list4mesh, list4mesh_node_t *prev_node, void *data) {
  assert(list4mesh != NULL);
  assert(prev_node != NULL);
  assert(data != NULL);

  list4mesh_node_t *node = (list4mesh_node_t *)list4mesh->allocator->alloc(sizeof(list4mesh_node_t));
  if (!node)
    return false;

    node->next = prev_node->next;
    node->data = data;
    prev_node->next = node;
    if (list4mesh->tail == prev_node) {
        list4mesh->tail = node;
    }
    ++list4mesh->length;
    return true;
}

bool list4mesh_prepend(list4mesh_t *list4mesh, void *data)
{
    assert(list4mesh != NULL);
    assert(data != NULL);

  list4mesh_node_t *node = (list4mesh_node_t *)list4mesh->allocator->alloc(sizeof(list4mesh_node_t));
    if (!node) {
        return false;
    }
    node->next = list4mesh->head;
    node->data = data;
    list4mesh->head = node;
    if (list4mesh->tail == NULL) {
        list4mesh->tail = list4mesh->head;
    }
    ++list4mesh->length;
    return true;
}

bool list4mesh_append(list4mesh_t *list4mesh, void *data)
{
    assert(list4mesh != NULL);
    assert(data != NULL);

  list4mesh_node_t *node = (list4mesh_node_t *)list4mesh->allocator->alloc(sizeof(list4mesh_node_t));
    if (!node) {
        return false;
    }
    node->next = NULL;
    node->data = data;
    if (list4mesh->tail == NULL) {
        list4mesh->head = node;
        list4mesh->tail = node;
    } else {
        list4mesh->tail->next = node;
        list4mesh->tail = node;
    }
    ++list4mesh->length;
    return true;
}

bool list4mesh_remove(list4mesh_t *list4mesh, void *data)
{
    assert(list4mesh != NULL);
    assert(data != NULL);

    if (list4mesh_is_empty(list4mesh)) {
        return false;
    }

    if (list4mesh->head->data == data) {
        list4mesh_node_t *next = list4mesh_free_node(list4mesh, list4mesh->head);
        if (list4mesh->tail == list4mesh->head) {
            list4mesh->tail = next;
        }
        list4mesh->head = next;
        return true;
    }

    for (list4mesh_node_t *prev = list4mesh->head, *node = list4mesh->head->next; node; prev = node, node = node->next)
        if (node->data == data) {
            prev->next = list4mesh_free_node(list4mesh, node);
            if (list4mesh->tail == node) {
                list4mesh->tail = prev;
            }
            return true;
        }

    return false;
}

void list4mesh_clear(list4mesh_t *list4mesh)
{
    assert(list4mesh != NULL);
    for (list4mesh_node_t *node = list4mesh->head; node; ) {
        node = list4mesh_free_node(list4mesh, node);
    }
    list4mesh->head = NULL;
    list4mesh->tail = NULL;
    list4mesh->length = 0;
}

list4mesh_node_t *list4mesh_foreach(const list4mesh_t *list4mesh, list4mesh_iter_cb callback, void *context)
{
  assert(list4mesh != NULL);
  assert(callback != NULL);

  for (list4mesh_node_t *node = list4mesh->head; node; ) {
    list4mesh_node_t *next = node->next;
    if (!callback(node->data, context))
      return node;
    node = next;
  }
  return NULL;
}

list4mesh_node_t *list4mesh_begin(const list4mesh_t *list4mesh)
{
    assert(list4mesh != NULL);
    return list4mesh->head;
}

list4mesh_node_t *list4mesh_end(UNUSED_ATTR const list4mesh_t *list4mesh)
{
    assert(list4mesh != NULL);
    return NULL;
}

list4mesh_node_t *list4mesh_next(const list4mesh_node_t *node)
{
    assert(node != NULL);
    return node->next;
}

void *list4mesh_node(const list4mesh_node_t *node)
{
    assert(node != NULL);
    return node->data;
}

list4mesh_node_t *list4mesh_free_node(list4mesh_t *list4mesh, list4mesh_node_t *node)
{
    assert(list4mesh != NULL);
    assert(node != NULL);

    list4mesh_node_t *next = node->next;

    if (list4mesh->free_cb) {
        list4mesh->free_cb(node->data);
    }
    list4mesh->allocator->free(node);
    --list4mesh->length;

    return next;
}
