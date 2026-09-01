#include "aboutdialog.h"
#include "ui_aboutdialog.h"

#include "buildinfo.h"

#include <boost/config.hpp>

AboutDialog::AboutDialog(QWidget * parent) : QDialog(parent), ui(new Ui::AboutDialog) {
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);

    ui->setupUi(this);
    ui->knossosLabel->setText(tr("KNOSSOS"));
    const QString branch = QString::fromUtf8(KBRANCH);
    ui->revisionLabelValue->setText(QString("%1%2").arg(KREVISION).arg(branch.isEmpty() ? QString{} : QString(" [%1]").arg(branch)));
    ui->revisionDateLabelValue->setText(KREVISIONDATE);
    ui->qtVersionLabelValue->setText(tr("using %1 (built with %2)").arg(qVersion()).arg(QT_VERSION_STR));

    ui->compilerLabelValue->setText(QString{"%1\n%2"}.arg(BOOST_COMPILER).arg(BOOST_STDLIB));
}

AboutDialog::~AboutDialog() {
    delete ui;
}
