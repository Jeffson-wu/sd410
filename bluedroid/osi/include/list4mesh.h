#ifndef _LIST4MESH_H_
#define _LIST4MESH_H_

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
struct list4mesh_node_t;
typedef struct list4mesh_node_t list4mesh_node_t;

struct list4mesh_t;
typedef struct list4mesh_t list4mesh_t;

typedef void (*list4mesh_free_cb)(void *data);
typedef bool (*list4mesh_iter_cb)(void *data, void *context);

// Returns a new, empty list4mesh. Returns NULL if not enough memory could be allocated
// for the list4mesh structure. The returned list4mesh must be freed with |list4mesh_free|. The
// |callback| specifies a function to be called whenever a list4mesh element is removed
// from the list4mesh. It can be used to release resources held by the list4mesh element, e.g.
// memory or file descriptor. |callback| may be NULL if no cleanup is necessary on
// element removal.
list4mesh_t *list4mesh_new(list4mesh_free_cb callback);


list4mesh_node_t *list4mesh_free_node(list4mesh_t *list4mesh, list4mesh_node_t *node);
// Frees the list4mesh. This function accepts NULL as an argument, in which case it
// behaves like a no-op.
void list4mesh_free(list4mesh_t *list4mesh);

// Returns true if |list4mesh| is empty (has no elements), false otherwise.
// |list4mesh| may not be NULL.
bool list4mesh_is_empty(const list4mesh_t *list4mesh);

// Returns true if the list4mesh contains |data|, false otherwise.
// |list4mesh| may not be NULL.
bool list4mesh_contains(const list4mesh_t *list4mesh, const void *data);

// Returns the length of the |list4mesh|. |list4mesh| may not be NULL.
size_t list4mesh_length(const list4mesh_t *list4mesh);

// Returns the first element in the list4mesh without removing it. |list4mesh| may not
// be NULL or empty.
void *list4mesh_front(const list4mesh_t *list4mesh);

// Returns the last element in the list4mesh without removing it. |list4mesh| may not
// be NULL or empty.
void *list4mesh_back(const list4mesh_t *list4mesh);
list4mesh_node_t *list4mesh_back_node(const list4mesh_t *list4mesh);

// Inserts |data| after |prev_node| in |list4mesh|. |data|, |list4mesh|, and |prev_node|
// may not be NULL. This function does not make a copy of |data| so the pointer
// must remain valid at least until the element is removed from the list4mesh or the
// list4mesh is freed. Returns true if |data| could be inserted, false otherwise
// (e.g. out of memory).
bool list4mesh_insert_after(list4mesh_t *list4mesh, list4mesh_node_t *prev_node, void *data);

// Inserts |data| at the beginning of |list4mesh|. Neither |data| nor |list4mesh| may be NULL.
// This function does not make a copy of |data| so the pointer must remain valid
// at least until the element is removed from the list4mesh or the list4mesh is freed.
// Returns true if |data| could be inserted, false otherwise (e.g. out of memory).
bool list4mesh_prepend(list4mesh_t *list4mesh, void *data);

// Inserts |data| at the end of |list4mesh|. Neither |data| nor |list4mesh| may be NULL.
// This function does not make a copy of |data| so the pointer must remain valid
// at least until the element is removed from the list4mesh or the list4mesh is freed.
// Returns true if |data| could be inserted, false otherwise (e.g. out of memory).
bool list4mesh_append(list4mesh_t *list4mesh, void *data);

// Removes |data| from the list4mesh. Neither |list4mesh| nor |data| may be NULL. If |data|
// is inserted multiple times in the list4mesh, this function will only remove the first
// instance. If a free function was specified in |list4mesh_new|, it will be called back
// with |data|. This function returns true if |data| was found in the list4mesh and removed,
// false otherwise.
//list4mesh_node_t list4mesh_remove_node(list4mesh_t *list4mesh, list4mesh_node_t *prev_node, list4mesh_node_t *node);
//list4mesh_node_t list4mesh_insert_node(list4mesh_t *list4mesh, list4mesh_node_t *prev_node, list4mesh_node_t *node);

bool list4mesh_remove(list4mesh_t *list4mesh, void *data);

// Removes all elements in the list4mesh. Calling this function will return the list4mesh to the
// same state it was in after |list4mesh_new|. |list4mesh| may not be NULL.
void list4mesh_clear(list4mesh_t *list4mesh);

// Iterates through the entire |list4mesh| and calls |callback| for each data element.
// If the list4mesh is empty, |callback| will never be called. It is safe to mutate the
// list4mesh inside the callback. If an element is added before the node being visited,
// there will be no callback for the newly-inserted node. Neither |list4mesh| nor
// |callback| may be NULL.
list4mesh_node_t *list4mesh_foreach(const list4mesh_t *list4mesh, list4mesh_iter_cb callback, void *context);

// Returns an iterator to the first element in |list4mesh|. |list4mesh| may not be NULL.
// The returned iterator is valid as long as it does not equal the value returned
// by |list4mesh_end|.
list4mesh_node_t *list4mesh_begin(const list4mesh_t *list4mesh);

// Returns an iterator that points past the end of the list4mesh. In other words,
// this function returns the value of an invalid iterator for the given list4mesh.
// When an iterator has the same value as what's returned by this function, you
// may no longer call |list4mesh_next| with the iterator. |list4mesh| may not be NULL.
list4mesh_node_t *list4mesh_end(const list4mesh_t *list4mesh);

// Given a valid iterator |node|, this function returns the next value for the
// iterator. If the returned value equals the value returned by |list4mesh_end|, the
// iterator has reached the end of the list4mesh and may no longer be used for any
// purpose.
list4mesh_node_t *list4mesh_next(const list4mesh_node_t *node);

// Returns the value stored at the location pointed to by the iterator |node|.
// |node| must not equal the value returned by |list4mesh_end|.
void *list4mesh_node(const list4mesh_node_t *node);

#endif /* _LIST4MESH_H_ */
