using System.Windows.Input;

namespace WarehouseApp.Commands
{
    internal class DelegateCommand : ICommand
    {
        private readonly Action execute;
        private readonly Func<bool>? canExecute;

        public DelegateCommand(Action execute, Func<bool>? canExecute = null)
        {
            this.execute = execute;
            this.canExecute = canExecute;
        }

#pragma warning disable CS0067
        public event EventHandler? CanExecuteChanged;
#pragma warning restore CS0067

        public bool CanExecute(object? parameter) => canExecute?.Invoke() == true;

        public void Execute(object? parameter) => execute();
    }
}
