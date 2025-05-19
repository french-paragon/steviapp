#include <QtTest/QtTest>

#include "vision/indexed_timed_sequence.h"

class TestIndexedTimeSequence: public QObject
{
    Q_OBJECT
private Q_SLOTS:
    void testGetValueAtTime();
    void testGetValuesInBetweenTimes();
};

void TestIndexedTimeSequence::testGetValueAtTime() {

    using TimedElement = StereoVisionApp::IndexedTimeSequence<double>::TimedElement;

    constexpr int nElements = 10;

    std::vector<TimedElement> data(nElements);

    for (int i = 0; i < nElements; i++) {
        data[i].time = i;
        data[i].val = i;
    }

    StereoVisionApp::IndexedTimeSequence<double> sequence(data);

    //check elements within the sequence
    for (int i = 0; i < nElements-1; i++) {

        std::array<double,5> dts = {0, 0.25, 0.5, 0.75};

        for (double dt : dts) {

            double time = i + dt;

            auto interpolable = sequence.getValueAtTime(time);

            double val = interpolable.weigthLower*interpolable.valLower + interpolable.weigthUpper*interpolable.valUpper;

            QCOMPARE(val, time);

        }

    }

    //check elements outside of the sequence (nearest interpolation is applied outside the sequence)
    std::array<double,5> dts = {0.25, 0.5, 0.75};

    for (double dt : dts) {

        double time0 = -dt;
        double timeN = nElements-1 + dt;

        auto interpolable0 = sequence.getValueAtTime(time0);
        auto interpolableN= sequence.getValueAtTime(timeN);

        double val0 = interpolable0.weigthLower*interpolable0.valLower + interpolable0.weigthUpper*interpolable0.valUpper;
        double valN = interpolableN.weigthLower*interpolableN.valLower + interpolableN.weigthUpper*interpolableN.valUpper;

        QCOMPARE(val0, 0);
        QCOMPARE(valN, nElements-1);
    }

}


void TestIndexedTimeSequence::testGetValuesInBetweenTimes() {

    using TimedElement = StereoVisionApp::IndexedTimeSequence<double>::TimedElement;

    constexpr int nElements = 10;

    std::vector<TimedElement> data(nElements);

    for (int i = 0; i < nElements; i++) {
        data[i].time = i;
        data[i].val = i;
    }

    StereoVisionApp::IndexedTimeSequence<double> sequence(data);

    double t0;
    double tf;
    double dt;

    //test get elements before the start of the sequence.
    t0 = -2.5;
    tf = 1.5;

    auto elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt >= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements at the center of the sequence.
    t0 = 2.5;
    tf = 6.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt >= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements after the end of the sequence.
    t0 = 7.5;
    tf = 11.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt >= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements in reverse direction before the start of the sequence
    t0 = 2.5;
    tf = -1.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt <= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements in reverse direction at the center of the sequence.
    t0 = 4.5;
    tf = 1.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt <= 0);
    }

    QCOMPARE(dt, tf-t0);

    //test get elements in reverse direction after the end of the sequence.
    t0 = 12.5;
    tf = 8.5;

    elements = sequence.getValuesInBetweenTimes(t0,tf);

    dt = 0;
    for (size_t i = 0; i < elements.size(); i++) {
        dt += elements[i].dt;
        QVERIFY(dt <= 0);
    }

    QCOMPARE(dt, tf-t0);

}

QTEST_MAIN(TestIndexedTimeSequence)
#include "testIndexedTimeSequence.moc"
