using System.Windows.Input;

namespace WarehouseApp.Commands
{
    internal class AsyncDelegateCommand : ICommand
    {
        private readonly Func<Task> execute;
        private readonly Func<bool>? canExecute;

        public AsyncDelegateCommand(Func<Task> execute, Func<bool>? canExecute = null)
        {
            this.execute = execute;
            this.canExecute = canExecute;
        }

#pragma warning disable CS0067
        public event EventHandler? CanExecuteChanged;
#pragma warning restore CS0067
        public bool CanExecute(object? parameter) => canExecute?.Invoke() == true;

        public async void Execute(object? parameter) => await execute();
    }
}
