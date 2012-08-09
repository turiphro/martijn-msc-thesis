/* Some functions using lists (used by other objects) */
#ifndef LISTHELPERS
#define LISTHELPERS

class ListHelper {

  public:

    int bool_find_next(bool* list, int count, int i, bool value=true)
    {
      while (i<count)
        if (list[++i] == value)
          return i;
      return -1;
    }

    int bool_find_first(bool* list, int count, bool value=true)
    {
      int i = -1;
      return bool_find_next(list, count, i, value);
    }

    int bool_find_prev(bool* list, int count, int i, bool value=true)
    {
      while (i >= 0)
        if (list[--i] == value)
          return i;
      return -1;
    }

    int bool_find_last(bool* list, int count, bool value=true)
    {
      int i = count;
      return bool_find_prev(list, count, i, value);
    }

};

#endif
