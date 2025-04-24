#ifndef INDEXED_TIMED_SEQUENCE_H
#define INDEXED_TIMED_SEQUENCE_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>

namespace StereoVisionApp {

/*!
 * \brief The IndexedTimeSequence class contain an indexed time sequence of elements T
 *
 * The class is meant to quickly access values in a time sequence.
 * It uses implicit sharing to make copying and moving data around more efficently.
 */
template<typename T, typename TimeT = double>
class IndexedTimeSequence {

public:

    struct TimedElement {
        TimeT time;
        T val;
    };

    /*!
     * \brief The TimeInterpolableVals struct contain a pair of values for interpolation
     *
     * The IndexedTimeSequence do not care about interpolation directly, as the end user might want to use specific approaches.
     * For example, some values might have to be interpolated on a manifold. So when asking for a value at a specific time,
     * the IndexedTimeSequence will return a pair of values with weights that the user can combine anyway they like.
     */
    struct TimeInterpolableVals {
        double weigthLower;
        T valLower;
        double weigthUpper;
        T valUpper;
    };

    struct Differential {
        double dt;
        T val;
    };

    /*!
     * \brief IndexedTimeSequence construct an empty sequence.
     */
    IndexedTimeSequence() :
        _initialTime(1),
        _finalTime(0),
        _indicesFromTimeIntervals(std::make_shared<std::vector<int>>()),
        _sequence(std::make_shared<std::vector<TimedElement>>())
    {

    }

    /*!
     * \brief IndexedTimeSequence copy constructor from vector
     * \param sequence
     */
    IndexedTimeSequence(std::vector<TimedElement> const& sequence) :
        _indicesFromTimeIntervals(std::make_shared<std::vector<int>>()),
        _sequence(std::make_shared<std::vector<TimedElement>>(sequence))
    {
        rebuildIndex();
    }

    /*!
     * \brief IndexedTimeSequence move constructor from vector
     * \param sequence the sequence.
     */
    IndexedTimeSequence(std::vector<TimedElement> && sequence) :
        _indicesFromTimeIntervals(std::make_shared<std::vector<int>>()),
        _sequence(std::make_shared<std::vector<TimedElement>>(sequence))
    {
        rebuildIndex();
    }

    /*!
     * \brief IndexedTimeSequence copy constructor from IndexedTimeSequence
     * \param other
     */
    IndexedTimeSequence(IndexedTimeSequence<T> const& other) :
        _initialTime(other._initialTime),
        _finalTime(other._finalTime),
        _indicesFromTimeIntervals(other._indicesFromTimeIntervals),
        _sequence(other._sequence)
    {

    }

    /*!
     * \brief IndexedTimeSequence move constructor from vector
     * \param sequence the sequence.
     */
    IndexedTimeSequence(IndexedTimeSequence<T> && other) :
        _initialTime(other._initialTime),
        _finalTime(other._finalTime),
        _indicesFromTimeIntervals(std::move(other._indicesFromTimeIntervals)),
        _sequence(std::move(other._sequence))
    {

    }

    IndexedTimeSequence<T, TimeT>& operator= (IndexedTimeSequence<T, TimeT> const& other) {
        _initialTime = other._initialTime;
        _finalTime = other._finalTime;
        _indicesFromTimeIntervals = other._indicesFromTimeIntervals;
        _sequence = other._sequence;
        return *this;
    }

    IndexedTimeSequence<T, TimeT>& operator= (IndexedTimeSequence<T, TimeT> && other) {
        _initialTime = other._initialTime;
        _finalTime = other._finalTime;
        _indicesFromTimeIntervals = std::move(other._indicesFromTimeIntervals);
        _sequence = std::move(other._sequence);
        return *this;
    }

    /*!
     * \brief rebuildIndex rebuild the index
     * \param nElements the number of elements to put in the index
     *
     * This function could break implicit sharing of the index data. It is not thread safe.
     */
    void rebuildIndex(int nElements = -1) {
        int nElems = nElements;

        if (nElems <= 0) {
            nElems = _sequence->size();
        }

        if (nElems == _indicesFromTimeIntervals->size()) {
            return;
        }

        //if the sequence is shared it might already be sorted, if not sorted, none of the other users indexed it so it is safe to sort.
        //it make the function not thread safe
        std::sort(_sequence->begin(), _sequence->end(), [] (TimedElement const& t1, TimedElement const& t2) -> bool {
            return t1.time < t2.time;
        });

        _initialTime = _sequence->front().time;
        _finalTime = _sequence->back().time;

        if (nElems != _indicesFromTimeIntervals->size()) {

            _indicesFromTimeIntervals = std::make_shared<std::vector<int>>(nElems); //decouple the index
        }

        TimeT deltaT = indexTimePerStep();

        int p = 0;

        for (int i = 0; i < nElems; i++) {

            while ((*_sequence)[p].time < _initialTime+i*deltaT) {
                p++;
            }

            (*_indicesFromTimeIntervals)[i] = p;
        }
    }

    TimeT indexTimePerStep() const {
        if (_indicesFromTimeIntervals->size() <= 1) {
            return 1; //by default return something for sequences with just one element.
        }
        TimeT deltaT = (_finalTime - _initialTime)/_indicesFromTimeIntervals->size();
        if (deltaT <= 0) {
            return 1;
        }
        return deltaT;
    }

    TimeInterpolableVals getValueAtTime(TimeT time) const {

        if (time <= _initialTime) {
            return TimeInterpolableVals{0, _sequence->front().val, 1, _sequence->front().val};
        }

        if (time >= _finalTime) {
            return TimeInterpolableVals{1, _sequence->back().val, 0, _sequence->back().val};
        }

        double deltaT = indexTimePerStep();
        double localTime = time - _initialTime;

        int initialIndexIdx = static_cast<int>(std::floor(localTime/deltaT));
        initialIndexIdx = std::clamp<int>(initialIndexIdx,0,_indicesFromTimeIntervals->size()-1);
        int initialSearchIdx = (*_indicesFromTimeIntervals)[initialIndexIdx];

        initialSearchIdx = std::clamp<int>(initialSearchIdx-1, 0, _sequence->size()-1);

        for (int i = initialSearchIdx; i < _sequence->size(); i++) {

            int nextId = std::min<int>(i+1,_sequence->size()-1);

            if ((*_sequence)[i].time <= time and (*_sequence)[nextId].time >= time) {
                double timeDelta = (*_sequence)[nextId].time - (*_sequence)[i].time;
                double preDelta = time - (*_sequence)[i].time;
                double postDelta = (*_sequence)[nextId].time - time;

                double wPre = preDelta/timeDelta;
                double wPost = postDelta/timeDelta;

                if (!std::isfinite(wPre) and !std::isfinite(wPost)) {
                    wPre = 0.5;
                    wPost = 0.5;
                }

                return {wPost, (*_sequence)[i].val, wPre, (*_sequence)[nextId].val};
            }

        }

        return TimeInterpolableVals{1, _sequence->back().val, 0, T()};
    }

    std::vector<Differential> getValuesInBetweenTimes(TimeT t0, TimeT tf) const {

        if (t0 >= _finalTime) {
            return std::vector<Differential>{Differential{tf - t0, _sequence->back().val}};
        }

        double deltaT = indexTimePerStep();
        double localT0 = t0 - _initialTime;

        int initialIndexIdx = static_cast<int>(std::floor(localT0/deltaT));

        if (initialIndexIdx < 0) {
            initialIndexIdx = 0;
        }

        int initialSearchIdx = (*_indicesFromTimeIntervals)[initialIndexIdx];

        initialSearchIdx = std::max(0, initialSearchIdx-1);

        int initialIndex = initialSearchIdx;

        while (initialIndex < _sequence->size() and (*_sequence)[initialIndex].time < t0) {
            initialIndex++;
        }

        int finalIndex = initialIndex;

        while (finalIndex < _sequence->size() and (*_sequence)[finalIndex].time < tf) {
            finalIndex++;
        }

        if (initialIndex > 0) {
            initialIndex--;
            finalIndex--;
        }

        //now all the elements we need to return as differentials are within initialIndex and finalIndex
        if (finalIndex == initialIndex) {
            return std::vector<Differential>{Differential{tf - t0, (*_sequence)[initialIndex].val}};
        }

        int nElements = finalIndex - initialIndex + 1;
        double t = t0;

        int currentIndex = initialIndex;

        std::vector<Differential> ret(nElements);

        for (int i = 0; i < nElements; i++) {

            if (currentIndex+1 > _sequence->size()) {
                ret[i] = Differential{tf - t, (*_sequence)[currentIndex].val};
                break;
            }

            if ((*_sequence)[currentIndex+1].time > tf) {
                ret[i] = Differential{tf - t, (*_sequence)[currentIndex].val};
                break;
            }

            TimeT dt = tf - t;
            if (currentIndex < finalIndex) {
                dt = (*_sequence)[currentIndex+1].time - t;
            }

            ret[i] = Differential{dt, (*_sequence)[currentIndex].val};

            t = (*_sequence)[currentIndex+1].time;
            currentIndex++;
        }

        return ret;

    }

    TimeT sequenceStartTime() const {
        return _initialTime;
    }

    TimeT sequenceEndTime() const {
        return _finalTime;
    }

    int nPoints() const {
        return _sequence->size();
    }

    TimedElement& operator[](int idx) {
        return (*_sequence)[idx];
    }

    TimedElement const& operator[](int idx) const {
        return (*_sequence)[idx];
    }

    bool isValid() const {
        return _initialTime <= _finalTime;
    }

protected:

    TimeT _initialTime;
    TimeT _finalTime;

    std::shared_ptr<std::vector<int>> _indicesFromTimeIntervals;
    std::shared_ptr<std::vector<TimedElement>> _sequence;

};

}

#endif // INDEXED_TIMED_SEQUENCE_H
