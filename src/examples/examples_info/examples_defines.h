#ifndef TESTS_DEFINES_
#define TESTS_DEFINES_

#ifdef __cplusplus
extern "C" {
#endif


typedef int (*example_ptr)(void);
example_ptr example_pointer;
void build_examples(void);

#ifdef __cplusplus
}
#endif


#endif
