#include <QtTest/QtTest>

#include "utils/functional.h"

class TestFunctionalUtils : public QObject
{
    Q_OBJECT
public:

private Q_SLOTS:

    void testTupleCaller();

};

void TestFunctionalUtils::testTupleCaller() {

    //test tuple manip
    std::tuple<int,double,int> strangeData(42,3.14159265,42);
    std::tuple<int,int> removed = StereoVisionApp::CallFromTuple::removeArgFromTuple<1>(strangeData);

    int charmed = 69;

    std::tuple<int,int&,int> charmedData = StereoVisionApp::CallFromTuple::insertArgInTuple<1,int&>(removed,charmed);

    QCOMPARE(std::get<1>(charmedData),69);

    charmed = 27;
    QCOMPARE(std::get<1>(charmedData),charmed);
    QCOMPARE(std::get<1>(charmedData),27);

    //test call to function
    std::get<0>(charmedData) = 33;
    std::get<2>(charmedData) = 6;

    auto callable = [] (int const& source, int& target, int retVal) {

        target = source;
        return retVal;
    };

    int val = StereoVisionApp::CallFromTuple::call(callable, charmedData);
    QCOMPARE(std::get<1>(charmedData),std::get<0>(charmedData));
    QCOMPARE(std::get<1>(charmedData),33);
    QCOMPARE(val,std::get<2>(charmedData));
    QCOMPARE(val,6);

}

QTEST_MAIN(TestFunctionalUtils);
#include "main.moc"
