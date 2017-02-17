namespace rtt {

template<typename T, typename U>
bool BiMap<T, U>::LeftComp::operator()(const LeftPair& a, const LeftPair& b) const {
    return a.first < b.first;
}
      
template<typename T, typename U>
bool BiMap<T, U>::RightComp::operator()(const RightPair& a, const RightPair& b) const {
    return a.first < b.first;
}
    
template<typename T, typename U>
bool BiMap<T, U>::insert(T& left, U& right) {
    if (containsLeft(left)) {
        return false;
    }
    LeftPair l = {left, &right};
    RightPair r = {right, &left};
    leftSet.insert(l);
    rightSet.insert(r);
    return true;
}
    
template<typename T, typename U>
unsigned int BiMap<T, U>::size() const {
    return leftSet.size();
}

template<typename T, typename U>
U* BiMap<T, U>::lookupLeft(const T& left) const {
    auto it = leftSet.find({left, nullptr});
    return it == leftSet.end() ? nullptr : it->second;
}
    
template<typename T, typename U>
T* BiMap<T, U>::lookupRight(const U& right) const {
    auto it = rightSet.find({right, nullptr});
    return it == rightSet.end() ? nullptr : it->second;
}    
   
template<typename T, typename U>
bool BiMap<T, U>::containsLeft(const T& left) {
    auto it = leftSet.find({left, nullptr});
    return it != leftSet.end();
}        
   
template<typename T, typename U>
bool BiMap<T, U>::containsRight(const U& right) {
    auto it = rightSet.find({right, nullptr});
    return it != rightSet.end();
}    
   
template<typename T, typename U>
bool BiMap<T, U>::removeByLeft(const T& left) {
    auto it = leftSet.find({left, nullptr});
    if (it == leftSet.end()) {
        return false;
    }
    rightSet.erase({*it->second, nullptr});
    leftSet.erase({left, nullptr});
    return true;
}    
    
template<typename T, typename U>
bool BiMap<T, U>::removeByRight(const U& right) {
    auto it = rightSet.find({right, nullptr});
    if (it == rightSet.end()) {
        return false;
    }
    leftSet.erase({*it->second, nullptr});
    rightSet.erase({right, nullptr});
    return true;
}

template<typename T, typename U>
typename std::set<typename BiMap<T, U>::LeftPair, typename BiMap<T, U>::LeftComp>::const_iterator BiMap<T, U>::begin() const {
    return leftSet.cbegin();
}


template<typename T, typename U>
typename std::set<typename BiMap<T, U>::LeftPair, typename BiMap<T, U>::LeftComp>::const_iterator BiMap<T, U>::end() const {
    return leftSet.cend();
}

/*
template<typename T, typename U>
BiMap<T, U>::Iterator::Iterator() {
    it = leftSet.begin();
}

template<typename T, typename U>
BiMap<T, U>::Iterator::Iterator(const Iterator& other) {
    it = other.it;
}

template<typename T, typename U>
typename BiMap<T, U>::Iterator& BiMap<T, U>::Iterator::operator=(const Iterator& other) {
    it = other.it;
    return *this;
}
    
template<typename T, typename U>
bool BiMap<T, U>::Iterator::operator==(const Iterator& other) const {
    return it == other.it;
}
    
template<typename T, typename U>
bool BiMap<T, U>::Iterator::operator!=(const Iterator& other) const {
    return it != other.it;
}

template<typename T, typename U>
typename BiMap<T, U>::Iterator::Pair& BiMap<T, U>::Iterator::operator*() {
    auto ptr = *it;
    Pair res(ptr.first, *ptr.second);
    return res;
}

template<typename T, typename U>
typename BiMap<T, U>::Iterator::Pair* BiMap<T, U>::Iterator::operator->() {
    return nullptr;
}
*/
}