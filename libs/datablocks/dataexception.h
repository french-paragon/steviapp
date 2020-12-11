#ifndef STEREOVISIONAPP_DATAEXCEPTION_H
#define STEREOVISIONAPP_DATAEXCEPTION_H

#include <QException>

class QObject;

namespace StereoVisionApp {

class DataBlock;
class Project;

class DataException : public QException
{
public:
	DataException(QString const& what, const QObject *block);
	DataException(DataException const& other);

	virtual ~DataException();

	void raise() const override { throw *this; }
	DataException *clone() const override { return new DataException(*this); }

	const char* what() const noexcept override;
	const DataBlock* block() const;
	const Project* project() const;

protected:

	const QObject* _block;
	char* _what;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_DATAEXCEPTION_H
